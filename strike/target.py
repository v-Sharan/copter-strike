from time import sleep
from threading import Thread


class Target:
    """
    Target Update Class
    """

    def __init__(self):
        self.target = [
            [-35.3563618, 149.1643810],
            [-35.3557274, 149.1642737],
            [-35.3552068, 149.1642147],
            [-35.3547518, 149.1641504],
        ]
        self.initial = [-35.3572762, 149.1645312]
        self.tlat = self.initial[0]
        self.tlon = self.initial[1]
        # self.thread = Thread(target=self.update_in_loop, daemon=True)
        # self.thread.start()

    def update_in_loop(self):
        sleep(1)
        for tar in self.target:
            sleep(5)
            self.tlat, self.tlon = tar[0], tar[1]

    def return_coords(self):
        return self.tlat, self.tlon


if __name__ == "__main__":
    tar = Target()
    while True:
        target = tar.return_coords()
        print(target)
        sleep(0.5)
