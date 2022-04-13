import os
import sys
sys.path.append('components/micropython-nxp/port/genhdr')
from auto_generate_qstr import gen_qstr

path = os.path.normpath(os.getcwd())
print('Generate Qstring in :',path)
headerfile = './components/micropython-nxp/port/genhdr/qstrdefs.generated.h'
gen_qstr(path=path,hash_len=2,headerfile=headerfile);