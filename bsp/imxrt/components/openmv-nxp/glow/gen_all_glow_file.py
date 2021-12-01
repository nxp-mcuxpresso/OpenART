import os 
import onnx 
import argparse 

# rebuild <model_name>.h, adding a input macro
def check_in_out(_in, _out):
    if 'input' in _in.lower():
        return _in, _out
    else:
        return _out, _in
        
def rebuild_header_file(file_name, net_name):
    with open(file_name, 'r+') as f:
        lines = f.readlines()
        # find where is the input / output node 
        idx = lines.index('// Placeholder address offsets within mutable buffer (bytes).\n')
        _input_name = lines[idx+1].split(' ')[1]
        _output_name = lines[idx+2].split(' ')[1]
        _input_name, _output_name = check_in_out(_input_name, _output_name)
        _input_macro = '#define INPUT_OFFSET %s \n'%_input_name 
        _output_macro = '#define OUTPUT_OFFSET %s \n'%_output_name 
        # define the input/output size macro
        idx_1 = lines.index('//   Name: "%s"\n'%(_input_name.replace('%s_'%net_name.upper(), '')))
        input_type = lines[idx_1 + 1]
        split_input_type = input_type[input_type.index('1 x') + 3:]
        input_size = split_input_type[:len(split_input_type)-2].lstrip().replace('x', '*')
        _input_size_macro = '#define INPUT_SIZE (%s * sizeof(float)) \n'%input_size 

        idx_2 = lines.index('//   Name: "%s"\n'%(_output_name.replace('%s_'%net_name.upper(), '')))
        output_type = lines[idx_2 + 1]
        split_output_type = output_type[output_type.index('1 x') + 3:]
        output_size = split_output_type[:len(split_output_type)-2].lstrip()
        _output_size_macro = '#define OUTPUT_SIZE (%s) \n'%output_size
        lines.insert(idx+3, _input_macro + _output_macro + _input_size_macro + _output_size_macro)
        f.seek(0)
        f.write(''.join(lines))
        f.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--img_path", help="image_path", type=str, default=r"C:\Users\nxf48054\Desktop\pytorch\glow_eg_lenet\images")
    parser.add_argument("--model", help="Caffe2: specify the path including the init_net.pb & predict_net.pb, ONNX / Keras: the model name", type=str, default="./models/model_01_0.04_0.99.h5")
    parser.add_argument("--tune", help="tune the model to get high acc", action="store_true")
    parser.add_argument("--output", help="where to store the output files", type=str, default='./output')
    parser.add_argument("--img_mode", help="the img's scope", choices=["0to1", "neg1to1", "0to255", "neg128to127"], type=str, default="neg1to1")
    parser.add_argument("--input_name", help="the name of the input tensor", type=str, default="input_1_01")
    parser.add_argument("--net_name", help="the name of the generated output files", type=str, default="Network")
    parser.add_argument("-dl", "--dynamic_load", help="call the glow model in dynamic mode", type=int, default=1)
    #parser.add_argument("")
    args, unknown = parser.parse_known_args() 

    model_name = args.model
    img_path = args.img_path
    out = args.output
    tune = args.tune 
    img_mode = args.img_mode
    input_name = args.input_name
    net_name = args.net_name

    _, extension = os.path.splitext(model_name)
    if extension == '.h5':
        from keras.models import load_model  
        import onnxmltools as onnx
        model = load_model(model_name)
        input_name = model.input.name
        # restore the input name to the onnx format
        _1, _2 = input_name.split(':')
        _2 = "%02d"%(int(_2) + 1)
        input_name = _1 + "_" + _2
        onnx_model = onnx.convert_keras(model, target_opset=9)
        model_name = model_name.replace('.h5', '.onnx')
        onnx.save_model(onnx_model, model_name)

    # mkdir 
    if not os.path.exists(out):
        os.mkdir(out)

    # to list all the img_path 
    img_path_list = ''
    for n in os.listdir(img_path):
        img_path_list += os.path.join(img_path, n) + " "
    classfier_cmd = img_path_list + "-image-mode %s -image-layout=NHWC -image-channel-order=BGR -model=%s -model-input-name %s -dump-profile=%s/profile.yml -onnx-define-symbol=None,1"%(img_mode, model_name, input_name, out)
    os.system("image-classifier.exe %s"%classfier_cmd)
    if tune:
        os.system()

    compile_cmd = "-model=%s -emit-bundle=%s/source -backend=CPU -target=arm -mcpu=cortex-m7 -float-abi=hard -load-profile %s/profile.yml -quantization-schema=symmetric_with_power2_scale -quantization-precision-bias=Int8 -use-cmsis -network-name=%s -dump-graph-DAG=%s/graph.dot -onnx-define-symbol=None,1"%(model_name, out, out, net_name, out)
    if not os.path.isfile(model_name):
        s = input("Input the model-input in : {input_node_name,float,[n,c,h,w]}")
        compile_cmd += s 
    os.system("model-compiler.exe %s"%compile_cmd)

    #rebuild_header_file(out + '/source/%s.h'%net_name, net_name)

    # dot to pdf 
    dot_line = '-Tpdf %s/graph.dot -o %s/graph.pdf -Nfontname="Times New Roman,"'%(out, out)
    os.system("dot.exe %s"%dot_line)
    print("All the files are generated to %s, please go to check" % out)

    if(args.dynamic_load):
        # get input and gen the bin 
        onnx_model = onnx.load_model(model_name)
        input_shape = onnx_model.graph.input[0].type.tensor_type.shape
        input_dims = [x.dim_value for x in input_shape.dim]
        input_dims[0] = 1
        output_shape = onnx_model.graph.output[0].type.tensor_type.shape
        output_dims = [x.dim_value for x in output_shape.dim]
        output_dims[0] = 1
        from object2bin import dumpobj
        output_dims_4 = [1] * 4
        output_dims_4[4 - len(output_dims):] = output_dims
        dumpobj(out + '/source', net_name, input_dims, output_dims_4)
    