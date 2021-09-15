'''
/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014-2016 Damien P. George
 * Copyright (c) 2016 Paul Sokolovsky
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
'''
from pyb import Pin
import cmm
import gc
def load():
    print('loading')
    fd = open('/cmm_cfg.csv')
    dict = {}
    for sLine in fd:
        if sLine[:2] == '//':
            continue # // is comment
        if sLine[-2] == '\r':
            sLine = sLine[:-2]
        else:
            sLine = sLine[:-1]
        lst = sLine.split(',')
        if len(lst) < 4:
            continue
        if lst[1] == '-':
            comboName = lst[0] + '.' + lst[2]
        else:
            comboName = lst[0] + '.' + lst[1] + '.' + lst[2]
        # value格式:(hint字符串, pin字符串，pin对象，当前属主
        try:
            pin = Pin(lst[4])
            dict[comboName] = (lst[3],lst[4], Pin(lst[4]), None)
        except:
            dict[comboName] = (lst[3],lst[4], None, None)
        del(lst)
        del(comboName)
    fd.close()
    cmm.add(dict)
    return dict
if __name__ == '__main__':
    dict = load()
