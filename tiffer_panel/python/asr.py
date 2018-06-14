import os
import sys
from aip import AipSpeech

APP_ID = '9613889'
API_KEY = 'XEjkM5h9OXYN0SN0EmCCc0Hu'
SECRET_KEY = 'bcc92f1d276d8cc10f322c5a5f64ca0e'

client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)

def save_result(res):
    f = open('../file/result', 'w')
    f.write(res)

def get_file_content(filePath):
    with open(filePath, 'rb') as fp:
        return fp.read()


if __name__ == "__main__":

    result = client.asr(get_file_content('../file/16k.wav'), 'wav', 16000, {
        'lan': 'zh',
    })
    print(result['result'][0])
    r = result['result'][0]
    save_result(r)
