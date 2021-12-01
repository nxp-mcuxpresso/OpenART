# OpenMV Unit Tests.
#
import os, sensor, gc

TEST_DIR    = "unittest"
TEMP_DIR    = "/sd/unittest/temp"
DATA_DIR    = "/sd/unittest/data"
SCRIPT_DIR  = "/sd/unittest/script"

if not (TEST_DIR in os.listdir("/sd")):
    raise Exception('Unittest dir not found!')

print("")
test_failed = False

def print_result(test, result):
    s = "Unittest (%s)"%(test)
    padding = "."*(60-len(s))
    print(s + padding + result)

for test in sorted(os.listdir(SCRIPT_DIR)):
    if test.endswith(".py"):
        test_result = "PASSED"
        test_path = "/".join((SCRIPT_DIR, test))
        try:
            f = open(test_path)
            exec(f.read())
            if unittest(DATA_DIR, TEMP_DIR) == False:
                f.close()
                raise Exception()
            f.close()
        except Exception as e:
            print(str(e))
            if "unavailable" in str(e):
                test_result =  "DISABLED"
            else:
                test_failed = True
                test_result =  "FAILED"
        print_result(test, test_result)
        gc.collect()

if test_failed:
    print("\nSome tests have FAILED!!!\n\n")
else:
    print("\nAll tests PASSED.\n\n")
