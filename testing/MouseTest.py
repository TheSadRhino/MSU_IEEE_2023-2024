from pynput import mouse


def mouseMove(x, y):

    print(f"x={x}, y={y}")


if __name__ == '__main__':
    listener = mouse.Listener(mouseMove)
    listener.run()
