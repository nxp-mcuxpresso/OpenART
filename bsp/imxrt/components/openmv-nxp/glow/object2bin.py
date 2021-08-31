import os
import lief 
import argparse 
import numpy as np 

import math 
def Align(array_len, align_size):
    aligned_size = math.ceil(array_len / align_size) * align_size
    padding_size = aligned_size - array_len
    return [padding_size, aligned_size]


def dumpobj(path, model_name, shape, out_shape):
    name = path + "/%s.o"%model_name
    if(os.system(r"C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe -std=c99 -g -mthumb --target=arm-arm-none-eabi -mcpu=cortex-m7 -fshort-enums -fshort-wchar -mfpu=fpv5-d16 -mfloat-abi=hard -Ofast rely.c %s -o %s.elf"%(name, model_name))):
            raise Exception('Build %s.elf failed'%model_name)

    # static build, not the RWPI, will put the global varible on the code tail, not rel by r9
    elf_name = model_name + ".elf"
    elf = lief.parse(elf_name)
    os.remove(elf_name)
    section_contents = []
    try:
        text_section = elf.get_section('ER_RO')
        text_addr = text_section.virtual_address
        text_contents = text_section.content
        section_contents += text_contents
    except:
        raise Exception("No ER_RO")

    main_func_off = elf.get_function_address(model_name) - 1 - text_addr # lsb is 1
    
    try:
        zi_section = elf.get_section('ER_ZI')
        zi_contents = zi_section.content
        zi_offset = len(section_contents)
        zi_len = zi_section.size 
        section_contents += zi_contents
    except:
        raise Exception("No ER_ZI")

    control_header_size = 68
    # the main_func of the model entry, and we put a 64bytes control block ahead, 
    # so the main_func_off must add 64
    main_func_off += control_header_size 

    # locate where the code use the ZI data, and relocate it
    zi_code_pos = elf.get_function_address('__aeabi_errno_addr') - 1 - text_addr 
    zi_pos = zi_code_pos + elf.get_symbol('__aeabi_errno_addr').size - zi_len
    assert(zi_offset == ((section_contents[zi_pos] | (section_contents[zi_pos+1]<<8) | (section_contents[zi_pos+2] << 16) | (section_contents[zi_pos+3] << 24)) - text_addr))
    zi_pos += control_header_size

    # set the zi section just after the ER_RO as the fake data section
    zi_offset += control_header_size

    # palceholder 
    section_contents = [0] * control_header_size + section_contents
    input_offset = 0
    # calc the total_input size 
    input_size = 1
    for v in shape:
        input_size *= v
    output_offset = input_size * 4 # float input 
    padding, weights_offset = Align(len(section_contents), 64) 
    # if not aligned, need to pad 0
    section_contents += [0] * padding
    entry_offset = main_func_off
    # parse the Network.h, get all the three size
    header_contents = open(path + "/%s.h"%model_name).readlines()
    pos = header_contents.index('// Memory sizes (bytes).\n')
    const_mem_size = eval(header_contents[pos+1].split(' ')[-1])  
    mut_mem_size = eval(header_contents[pos+2].split(' ')[-1])
    act_mem_size = eval(header_contents[pos+3].split(' ')[-1])

    struct_header = [input_offset, output_offset, weights_offset, entry_offset,
                     mut_mem_size, act_mem_size, const_mem_size] + shape + out_shape + [zi_pos, zi_offset] # padding to control_header_size bytes 
    # need to transfer to uint32, and adheare to the head 
    struct_header_32 = list(np.asarray(struct_header, dtype='uint32').tobytes())
    section_contents[:control_header_size] = struct_header_32
    
    # add the weights 
    with open(path + "/%s.weights.bin"%model_name, 'rb') as f:
        section_contents += list(f.read())

    with open(model_name + '.glow', 'wb') as f:
        f.write(np.asarray(section_contents, dtype='uint8').tobytes())
        f.close()

    with open(model_name + '.h', 'w') as f:
        f.write("#include <stdint.h> \n")
        f.write('#include "glow_bundle.h" \n')
        f.write("#define MODEL_ENTRY_OFFSET %d\n"%main_func_off)
        f.write("#define MODEL_PTR ((void (*)(uint8_t*, uint8_t*, uint8_t*))(%s + MODEL_ENTRY_OFFSET))\n"%model_name)
        f.write("__attribute__((aligned(64))) uint8_t %s[] = {\n\t"%model_name)
        for i, v in enumerate(section_contents):
            f.write('0x%02x, '%v)
            if((i+1) % 16) == 0 :
                f.write("\n\t");
        f.write("};\n")
        f.close()
    print("obj2bin Done!")
    return
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", '--folder', help='objfile location', default='./output/source', type=str)
    args, unknown = parser.parse_known_args() 

    folder_name = args.folder
    dumpobj(folder_name, 'Network', [1, 28, 28, 1], [1, 1, 1, 10])