class Step:

    callback: any
    params: tuple
    complete: bool

    def __init__(self, callback, *callback_params):
        self.callback = callback
        self.params = callback_params
        self.complete = False

    def update(self):
        self.complete = self.callback()
