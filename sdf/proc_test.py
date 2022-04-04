import subprocess
import json
import shutil
import sys

assert len(sys.argv) == 2 and sys.argv[1].endswith(".exe")
build_json_raw = subprocess.run(
        ["cargo", "test", "--no-run", "--message-format=json"],
        stdout=subprocess.PIPE,
        text=True)
for token in build_json_raw.stdout.split("\n")[:-1]:
    token_obj = json.loads(token)
    if "profile" in token_obj.keys() and token_obj["profile"]["test"]:
        for fname in token_obj["filenames"]:
            if fname.endswith(".exe"):
                shutil.copyfile(fname, sys.argv[1])
            else:
                shutil.copyfile(fname, sys.argv[1][:-4] + ".pdb")