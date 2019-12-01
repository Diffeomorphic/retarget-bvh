# ------------------------------------------------------------------------------
#   BSD 2-Clause License
#   
#   Copyright (c) 2019, Thomas Larsson
#   All rights reserved.
#   
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#   
#   1. Redistributions of source code must retain the above copyright notice, this
#      list of conditions and the following disclaimer.
#   
#   2. Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution.
#   
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ------------------------------------------------------------------------------



import json
import gzip

def loadJson(filepath):
    try:
        with gzip.open(filepath, 'rb') as fp:
            bytes = fp.read()
    except IOError:
        bytes = None

    if bytes:
        string = bytes.decode("utf-8")
        struct = json.loads(string)
    else:
        with open(filepath, "r") as fp:
            struct = json.load(fp)

    return struct


def saveJson(struct, filepath, binary=False):
    if binary:
        bytes = json.dumps(struct)
        with gzip.open(realpath, 'wb') as fp:
            fp.write(bytes)
    else:
        string = encodeJsonData(struct, "")
        with open(filepath, "w", encoding="utf-8") as fp:
            fp.write(string)
            fp.write("\n")


def encodeJsonData(data, pad=""):
    if data == None:
        return "none"
    elif isinstance(data, bool):
        if data == True:
            return "true"
        else:
            return "false"
    elif isinstance(data, float):
        if abs(data) < 1e-6:
            return "0"
        else:
            return "%.5g" % data
    elif isinstance(data, int):
        return str(data)
    elif isinstance(data, str):
        return "\"%s\"" % data
    elif isinstance(data, (list, tuple)):
        if data == []:
            return "[]"
        elif leafList(data):
            string = "["
            for elt in data:
                string += encodeJsonData(elt) + ", "
            return string[:-2] + "]"
        else:
            string = "["
            for elt in data:
                string += "\n    " + pad + encodeJsonData(elt, pad+"    ") + ","
            return string[:-1] + "\n%s]" % pad
    elif isinstance(data, dict):
        if data == {}:
            return "{}"
        string = "{"
        for key,value in data.items():
            string += "\n    %s\"%s\" : " % (pad, key) + encodeJsonData(value, pad+"    ") + ","
        return string[:-1] + "\n%s}" % pad


def leafList(data):
    for elt in data:
        if isinstance(elt, (list,tuple,dict)):
            return False
    return True
