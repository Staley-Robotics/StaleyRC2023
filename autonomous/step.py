class Step:

    callback: any
    complete: bool

    def __init__(self, callback):
        self.callback = callback
        self.complete = False

    def update(self):
        self.complete = self.callback()
