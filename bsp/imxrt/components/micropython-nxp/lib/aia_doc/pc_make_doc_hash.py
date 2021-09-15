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
import time
def compute_hash(key, modulo=100):
    hash = 5381
    bKey = key.encode()
    for b in bKey:
        hash = (hash * 33) ^ b
    return hash % modulo

def open_helpfile(key):
    hash = compute_hash(key)
    sPath = '/aia_doc/zzhsh_%02d.md' % hash
    fd = open(sPath, 'ab')

import os
import os.path as path

def modmain():

    def OpenOutFile(sLine):
        key = sLine[5:]
        hash = compute_hash(key)
        sOutPath = 'zzhsh_%02d.md' % hash
        fdOut = open(sOutPath, 'ab')
        return fdOut, key

    os.system('del zzhsh_*.md')

    for root, dirs, files in os.walk('.', topdown=False):
        for file in files:
            if file[-3:] != '.md' or file[:6] == 'zzhsh_':
                continue
            sPath = path.join(root, file)
            fd = open(sPath, encoding='utf-8')
            s = fd.read()
            lst = s.split('\n')
            fd.close()
            state = 0
            for sLine in lst:
                if state == 0:
                    if sLine[:5] == '#### ':
                        state = 1
                        fdOut, key = OpenOutFile(sLine)
                if state == 1:
                    if sLine[:5] == '#### ':
                        if sLine[5:] != key:
                            fdOut.close()
                            fdOut,key = OpenOutFile(sLine)
                    fdOut.write((sLine+'\r\n').encode())


if __name__ == '__main__':
    #help()
    modmain()
