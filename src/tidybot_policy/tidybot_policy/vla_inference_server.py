"""
VLA Inference Server (ZMQ-based).
Runs on the GPU machine (WonderPad). Fetches camera images from the robot
via ZMQ (over SSH tunnel), runs OpenVLA inference, and sends actions back.

Usage:
    # First, create SSH tunnel from WonderPad to bane:
    ssh -L 5555:localhost:5555 janchen@bane

    # Then run inference:
    python vla_inference_server.py \
        --merged_ckpt /path/to/checkpoint \
        --instruction "open the drawer"
"""
from transformers import AutoModelForVision2Seq, AutoProcessor
from PIL import Image as PILImage
from peft import PeftModel

import zmq
import numpy as np
import torch
import cv2
import json
import os
import time
import argparse


def load_model(args):
    """Load VLA model and processor."""
    if args.merged_ckpt:
        print(f'Loading merged checkpoint from {args.merged_ckpt}')
        processor = AutoProcessor.from_pretrained(args.merged_ckpt, trust_remote_code=True)
        vla = AutoModelForVision2Seq.from_pretrained(
            args.merged_ckpt,
            torch_dtype=torch.bfloat16,
            low_cpu_mem_usage=True,
            trust_remote_code=True
        ).to("cuda:0")
        ckpt_dir = args.merged_ckpt
    else:
        print(f'Loading base model + adapter from {args.adapter_ckpt}')
        processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
        base_model = AutoModelForVision2Seq.from_pretrained(
            "openvla/openvla-7b",
            torch_dtype=torch.bfloat16,
            low_cpu_mem_usage=True,
            trust_remote_code=True
        ).to("cuda:0")
        vla = PeftModel.from_pretrained(base_model, args.adapter_ckpt).to("cuda:0")
        ckpt_dir = args.adapter_ckpt

    # Load dataset statistics
    stats_path = os.path.join(ckpt_dir, 'dataset_statistics.json')
    if os.path.exists(stats_path):
        with open(stats_path, 'r') as f:
            stats = json.load(f)
        if 'tidybot_vla' in stats:
            stats = stats['tidybot_vla']
        vla.config.norm_stats['tidybot_vla'] = stats
        print(f'Loaded dataset statistics from {stats_path}')
    else:
        print(f'WARNING: No dataset_statistics.json found at {stats_path}')

    return processor, vla


def main():
    parser = argparse.ArgumentParser(description='VLA Inference Server (ZMQ)')
    parser.add_argument('--merged_ckpt', type=str, default='',
                        help='Path to fully merged checkpoint')
    parser.add_argument('--adapter_ckpt', type=str, default='',
                        help='Path to LoRA adapter checkpoint')
    parser.add_argument('--instruction', type=str, default='pick up the object',
                        help='Language instruction for the robot')
    parser.add_argument('--inference_hz', type=float, default=5.0,
                        help='Inference frequency in Hz')
    parser.add_argument('--publish_hz', type=float, default=15.0,
                        help='Action publish frequency in Hz')
    args = parser.parse_args()

    if not args.merged_ckpt and not args.adapter_ckpt:
        parser.error('Must provide either --merged_ckpt or --adapter_ckpt')

    # Load model
    processor, vla = load_model(args)
    prompt = f"In: What action should the robot take to {args.instruction}?\nOut:"
    print(f'Using prompt: {prompt}')

    # Connect to ZMQ bridge on bane (via SSH tunnel)
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect('tcp://localhost:5555')
    print('Connected to ZMQ bridge at localhost:5555')

    inference_interval = 1.0 / args.inference_hz
    publish_interval = 1.0 / args.publish_hz
    latest_action = np.zeros(7)

    print('Starting inference loop...')
    last_inference_time = 0

    while True:
        try:
            now = time.time()

            # Run inference at inference_hz
            if now - last_inference_time >= inference_interval:
                # Request observation from robot
                socket.send_pyobj({'get_obs': True})
                rep = socket.recv_pyobj()

                if rep['status'] == 'ok':
                    # Decode compressed image
                    image_bytes = rep['image']
                    np_arr = np.frombuffer(image_bytes, np.uint8)
                    image_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
                    pil_image = PILImage.fromarray(image_rgb)

                    # Run VLA inference
                    inputs = processor(prompt, pil_image, return_tensors="pt").to(
                        "cuda:0", dtype=torch.bfloat16
                    )
                    with torch.no_grad():
                        action = vla.predict_action(
                            **inputs, unnorm_key="tidybot_vla", do_sample=False
                        )
                    latest_action = action
                    print(f'Inferred action: {action}')

                    # Send action to robot
                    socket.send_pyobj({'action': action})
                    socket.recv_pyobj()  # Wait for ack

                    last_inference_time = now
                elif rep['status'] == 'no_image':
                    print('Waiting for camera image...')
                    time.sleep(0.5)
            else:
                # Between inference steps, re-send latest action at publish_hz
                socket.send_pyobj({'action': latest_action})
                socket.recv_pyobj()
                time.sleep(publish_interval)

        except KeyboardInterrupt:
            print('Shutting down...')
            break
        except Exception as e:
            print(f'Error: {e}')
            import traceback
            traceback.print_exc()
            time.sleep(1.0)


if __name__ == '__main__':
    main()
