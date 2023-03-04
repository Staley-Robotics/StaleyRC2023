class Step:

    callback: any
    params: tuple
    complete: bool

    """
    Options for Autonomous strategy:
    1. Literally just exit the community
    2. Place a cone or cube on stand
    3. Try to balance on the Charging Station (prob don't use without pigeon)
    """

    def __init__(self, callback, *callback_params):
        self.callback = callback
        self.params = callback_params
        self.complete = False

