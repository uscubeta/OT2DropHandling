import csv

load_time_s = 30  # estimated time for 'load' action, in seconds
mix_time_s = 20  # estimated time for 'mix' action, in seconds
rinse_time_s = 60  # estimated time for 'rinse' or 'unload' action
start_time = 30  # seconds, when first action will run
load_mixes = 3  # number of times to mix when loading

class ActClass:
    def __init__(self, sam_id: int, action: str, stamp: int):
        # initializing function, _attribute is a hidden attribute !
        self._sam_id = sam_id  # integer of samples indexed 0 to num_sam
        # in the order entered by user_config_exp
        self._action = action  # string ['load', 'unload', 'reload', 'mix', 'rinse' ]
        self._start_stamp = stamp  # integer of start timestamp (seconds) for the desired action
        self._end_stamp = self._calc_end()  # calculated, estimated end timestamp
        self._which_tip = (0, 0)  # change to (rk,wl) terminology
        self._time_stamp = (self._start_stamp, self._end_stamp)


    # returns this when calling this object
    def __repr__(self):

        this_string = "(" + str(self.sam_id) + ", '" + \
                      str(self.action) + "', " + str(self.start) + \
                      ", " + str(self.end) + ")"
        return this_string

    # returns this string when called via print(x)
    def __str__(self):

        this_string = "(" + str(self.sam_id) + ", '" + \
                      str(self.action) + "', " + str(self.start) + \
                      ", " + str(self.end) + ")"
        return this_string

    # setter methods
    def change_start(self, new_start: int):
        self._start_stamp = new_start
        self._end_stamp = self._calc_end()
        self._time_stamp = (self._start_stamp, self._end_stamp)

    def change_tip(self, set_tip: (int, int)):
        self._which_tip = set_tip

    # property/ getter methods
    @property
    def tip_id(self):
        return self._which_tip

    @property
    def sam_id(self):
        return self._sam_id

    @property
    def action(self):
        return self._action

    @property
    def start(self):
        return self._start_stamp

    @property
    def end(self):
        return self._end_stamp

    @property
    def stamp(self):
        return self._time_stamp

    @property
    def length(self):
        time2complete = self._end_stamp - self._start_stamp
        return time2complete

    # other functions
    def _calc_end(self):
        # MODIFY: check that global variables are defined.
        if self.action == 'load':
            prev_end_stamp = self._start_stamp + load_time_s
        elif self.action == 'mix':
            prev_end_stamp = self._start_stamp + mix_time_s
        elif self.action == 'rinse' or self.action == 'unload' or self.action == 'reload':
            prev_end_stamp = self._start_stamp + rinse_time_s
        else:
            prev_end_stamp = self._start_stamp
        return prev_end_stamp


# with open('people.csv', 'r') as file:
#     reader = csv.reader(file, delimiter='\t')
#     for row in reader:
#         print(row)


# with open('protagonist.csv', 'w', newline='') as file:
#     writer = csv.writer(file)
#     writer.writerow(["SN", "Movie", "Protagonist"])
#     writer.writerow([1, "Lord of the Rings", "Frodo Baggins"])
#     writer.writerow([2, "Harry Potter", "Harry Potter"])

# with open('protagonist.csv', 'r') as file:
#     reader = csv.reader(file)
#     for row in reader:
#         print(row)

# with open('protagonist2.csv', 'w') as file:
#     writer = csv.writer(file, delimiter = '\t')
#     writer.writerow(["SN", "Movie", "Protagonist"])
#     writer.writerow([1, "Lord of the Rings", "Frodo Baggins"])
#     writer.writerow([2, "Harry Potter", "Harry Potter"])

with open("people.csv", 'r') as file:
    csv_file = csv.DictReader(file)
    for row in csv_file:
        print(dict(row))


with open('action_sequence.csv', 'w') as file:
    writer = csv.writer(file)
    writer.writerow(["sam_id", "action", "start_stamp", "end_stamp"])
    for action in exp.planned_sequence:
        writer.writerow([action.keeper, action.action, action.start, action.end])

with open('action_sequence.csv', 'w') as file:
