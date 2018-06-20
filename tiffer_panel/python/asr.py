#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
from aip import AipSpeech

APP_ID = '9613889'
API_KEY = 'XEjkM5h9OXYN0SN0EmCCc0Hu'
SECRET_KEY = 'bcc92f1d276d8cc10f322c5a5f64ca0e'

client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)

def save_result(res):
    f = open('/home/tiffer/tiffer-catkin/src/tiffer_panel/file/result', 'w')
    f.write(res)
    f.close()

def get_file_content(filePath):
    with open(filePath, 'rb') as fp:
        return fp.read()

if __name__ == "__main__":

    result = client.asr(get_file_content('/home/tiffer/tiffer-catkin/src/tiffer_panel/file/l'), 'wav', 16000, {
        'dev_pid': '1536',
    })

    try:
        if result['err_no'] == 0:
            r = result['result']
            save_result(r[0])
        else:
            print(result['err_msg'])
    except IOError as e:
        print(e)
