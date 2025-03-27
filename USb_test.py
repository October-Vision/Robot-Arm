import usb.core
import usb.util

# 替换为您的设备的 idVendor 和 idProduct
idVendor = 0x28E9  # 示例值
idProduct = 0x018A  # 示例值

device = usb.core.find(idVendor=idVendor, idProduct=idProduct)
if device is None:
    print("设备未找到")
else:
    print("设备已连接")