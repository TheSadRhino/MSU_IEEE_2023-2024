import evdev
from evdev import ecodes


def print_event(e):
    if e.type == ecodes.EV_SYN:
        if e.code == ecodes.SYN_MT_REPORT:
            msg = 'time {:<16} +++++++++ {} ++++++++'
        else:
            msg = 'time {:<16} --------- {} --------'
        print(msg.format(e.timestamp(), ecodes.SYN[e.code]))
    else:
        if e.type in ecodes.bytype:
            codename = ecodes.bytype[e.type][e.code]
        else:
            codename = '?'

        evfmt = 'time {:<16} type {} ({}), code {:<4} ({}), value {}'
        print(evfmt.format(e.timestamp(), e.type, ecodes.EV[e.type], e.code, codename, e.value))


devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

for device in devices:
    if device.name != "PixArt USB Optical Mouse":
        devices.remove(device)

leftMouse = None
rightMouse = None

while leftMouse is None:
    for device in devices:
        event = device.read_one()
        if event is not None:
            if event.code == 272:
                leftMouse = device
                devices.remove(device)
                break

rightMouse = devices[0]

while True:
    rightEvent = rightMouse.read_one()
    if rightEvent is not None:
        print("Right Mouse Event: ", end="")
        print_event(rightEvent)

    leftEvent = leftMouse.read_one()
    if leftEvent is not None:
        print("Left Mouse Event: ", end="")
        print_event(leftEvent)
