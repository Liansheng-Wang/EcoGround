import keyboard
while True:
    event = keyboard.read_event()
    print(event.name, event.event_type)

