import base64
import json
import numpy as np

def Json_Updata(file_path, key, value):
    #保存机械臂当前位置
    with open(file_path,'r',encoding='utf-8') as f:
        config = json.load(f)
    #替换值
    config[key] = value
    #将更新后的配置写入文件
    with open(file_path,'w',encoding='utf-8') as f:
        json.dump(config,f,ensure_ascii=False,indent=4)