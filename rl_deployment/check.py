import onnx
model = onnx.load("policy.onnx")
for input in model.graph.input:
    print(f"Input Name: {input.name}")
    # This might just say 'obs', but let's check metadata
    for prop in model.metadata_props:
        print(f"Metadata: {prop.key} : {prop.value}")
