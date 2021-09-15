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

def compute_hash(key, modulo=17):
    hash = 5381
    bKey = key.encode()
    for b in bKey:
        hash = (hash * 33) ^ b
    return hash % modulo

def open_helpfile(key):
    hash = compute_hash(key)
    try:
        sPath = '/aia_doc/zzhsh_%02d.md' % hash
        fd = open(sPath, 'rb')
    except:
        sPath = '/flash/aia_doc/zzhsh_%02d.md' % hash
        fd = open(sPath, 'rb')    
    return fd

def modmain():
    t0 = time.ticks()
    try:
        import doc
        blob = doc.doc()  # get the (key, lang) tuple
    except:
        blob = ('welcome', 'chs')
    key = blob[0]
    lang = blob[1]
    print('\033[0;0m', end='')  # start of doc help entry
    print('='*20,'\033[0;45;4m', end='') # pink
    print(key, end='')
    print('\033[0m','='*20)  # no color
    hash = compute_hash(key, 17)
    READ_LEN = 4096
    fd = open_helpfile(key)
    s = fd.read(READ_LEN)

    while len(s) > 1:
        subLen = len(s)
        ret = doc.search(s, subLen)
        if ret == 1:
            break # found
        elif ret == 2: # link
            blob = doc.doc() # get new key
            key = blob[0]
            fd.close()
            fd = open_helpfile(key)
            print('-->', key)
            fd.seek(0)
        elif ret < 0:            
            fd.seek(ret,1)
        s = fd.read(READ_LEN)
        if len(s) == -ret:
            break
    print('\033[8;0m')  # end of doc help entry
    t1 = time.ticks()
    #print('ticks = %d' % (t1 - t0))
    fd.close()

if __name__ == '__main__':
    #help()
    modmain()
