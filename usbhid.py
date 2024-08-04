Import("env")

print("------- START usbhid.py --------")

#print(env.Dump())

#'hwids': [['0x1B4F', '0x9206'], ['0x1B4F', '0x9205'], ['0x2341', '0x8037'], ['0x2341', '0x0037']]
#'usb_product': 'SparkFun Pro Micro'

board_config = env.BoardConfig()
# should be array of VID:PID pairs
#board_config.update("build.hwids", [["0x046D", "0xC62B"]])
#board_config.update("build.hwids", [["0x046D", "0xC62B"], ['0x1B4F', '0x9205'], ['0x2341', '0x8037'], ['0x2341', '0x0037']])

board_config.update("build.hwids", [["0x256f", "0xc635"], ['0x1B4F', '0x9205'], ['0x2341', '0x8037'], ['0x2341', '0x0037']])
board_config.update("build.usb_product", "3Dconnexion")
board_config.update("build.usb_manufacturer", "Gruffware")
board_config.update("build.vendor", "Gruffware")

board_config.update("vendor", "Gruffware")

# Use dir() to get all attributes and methods of the board_config object
attributes = dir(board_config)

# Filter out internal attributes and methods (those starting with '__')
attributes = [attr for attr in attributes if not attr.startswith('__')]

# Iterate and print all attributes
for attr in attributes:
    value = getattr(board_config, attr)
    if not callable(value):
        print(f"{attr}: {value}")

print("------- END usbhid.py --------")