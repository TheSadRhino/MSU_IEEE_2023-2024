# import usb
# import usb.backend.libusb1
#
# backend = usb.backend.libusb1.get_backend(find_library=lambda
#                                 x: "libusb-1.0.26-binaries\\libusb-1.0.26-binaries\\VS2015-x64\\dll\\libusb-1.0.dll")
# devices = usb.core.find(backend=backend, find_all=True)
#
# for dev in devices:
#     print("Bus:", dev.bus)
#     print("Type:", dev.)
#     print("  idVendor: %d (0x%04x)" % (dev.idVendor, dev.idVendor))
#     print("  idProduct: %d (0x%04x)" % (dev.idProduct, dev.idProduct))

import usb
busses = usb.busses()
for bus in busses:
    devices = bus.devices
    for dev in devices:
        print("Device:", dev.filename)
        print("Device Type:", dev.usbVersion)
        print("  idVendor: %d (0x%04x)" % (dev.idVendor, dev.idVendor))
        print("  idProduct: %d (0x%04x)" % (dev.idProduct, dev.idProduct))
