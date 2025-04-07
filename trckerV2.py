import pandas as pn
import numpy as np
import time
import pandas as pn

class KalmanFilter:
    def __init__(self, dt, state_dim=4, measurement_dim=2):
        self.dt = dt  # Time step
        self.A = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])  # State transition matrix
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])  # Measurement matrix
        self.Q = np.eye(state_dim) * 0.01  # Process noise covariance
        # self.R = np.eye(measurement_dim) * 1.0  # Measurement noise covariance
        self.R = np.eye(measurement_dim) * 1.0 # Measurement noise covariance
        # self.R = np.eye(measurement_dim) * 100
        self.P = np.eye(state_dim) * 1.0  # Covariance matrix

    def predict(self, state):
        """
        Predict the next state.
        :param state: Current state [x, y, vx, vy]
        :return: Predicted state [x, y, vx, vy]
        """
        predicted_state = np.dot(self.A, state)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return predicted_state

    def update(self, state, measurement):
        """
        Update the state based on measurement.
        :param state: Current state [x, y, vx, vy]
        :param measurement: Measurement [x, y]
        :return: Updated state [x, y, vx, vy]
        """
        y = measurement - np.dot(self.H, state)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        updated_state = state + np.dot(K, y)
        self.P = np.dot((np.eye(4) - np.dot(K, self.H)), self.P)
        return updated_state


class Tracker:
    def __init__(self, dt=0.01, max_age=10000, merge_threshold=10):
        self.dt = dt  # Time step
        self.max_age = max_age  # Maximum age of a track
        self.merge_threshold = merge_threshold  # Maximum distance for merging tracks
        self.tracks = []  # List to hold tracks
        self.track_id = 1  # Unique ID for tracks
        self.kf = KalmanFilter(dt=self.dt)  # Kalman Filter
        self.consecutive_mismatches_threshold = 5  # Threshold for consecutive mismatches
        # self.deletion_threshold = 3.0  # Threshold for deletion based on distance
        self.deletion_threshold = 0.5  # Threshold for position change
        self.consecutive_deletion_threshold = 5  # Threshold for consecutive mismatches for deletion
        self.reason=[]
        self.reason_id=[]


    def update(self, measurements):
        """
        Update the tracks based on measurements.
        :param measurements: List of measurements [x, y]
        """
        self.predict()
        self.assign(measurements)
        self.unassign()
        self.delete()

    def predict(self):
        """
        Predict the next state for each track.
        """
        for track in self.tracks:
            track.state = self.kf.predict(track.state)

    def assign(self, measurements):
        """
        Assign measurements to tracks.
        :param measurements: List of measurements [x, y]
        """
        assigned_measurements = []
        for track in self.tracks:
            if track.age == 0:
                continue  # Skip newly created tracks
            min_dist = np.inf
            best_measurement = None
            for m in measurements:

                # print(np.array(m[1]))
                gx=track.state[:2]
                # print(gx[1])
                # dist = np.linalg.norm(np.array(track.state[:2]) - np.array(m))
                # print(np.array(track.state[:2]))
                dist = abs(gx[1] - np.array(m[1]))
                # dist = np.linalg.norm(np.array(track.state[:2]) - m[1])
                if dist < min_dist:
                    min_dist = dist
                    best_measurement = m
            # print('merge',self.merge_threshold)
            # if best_measurement is not None and min_dist <= self.merge_threshold:
            # if best_measurement is not None and min_dist <= 10:
            # 10,5
            if best_measurement is not None and min_dist <= 8:
                    track.state = self.kf.update(track.state, best_measurement)  # Update state
                    track.measurement = best_measurement
                    track.first_state = best_measurement[1]
                    assigned_measurements.append(best_measurement)

        # Create new tracks for unassigned measurements
        unassigned_measurements = [m for m in measurements if m not in assigned_measurements]
        for um in unassigned_measurements:
            new_track = Track(self.track_id, um)
            self.track_id += 1

            # print('first_state',self.first_state)
            self.tracks.append(new_track)

        # Age tracks
        for track in self.tracks:
            track.age += 1

    def unassign(self):
        """
        Unassign tracks if they haven't been assigned for a while.
        """
        for track in self.tracks:
            if track.age > self.max_age:
                track.measurement = None
    def delete(self):
        tracks_to_delete = []
        for track in self.tracks:
            # print('s1',track.state[1])
            # print('s0', track.state[0])
            # # print('s2', track.state_history)
            #
            # print('s2',track.state[1]-track.formery )
            # print('counter', track.counter)
            # print('s3',track.state[0] - track.formerx)

            if (abs(track.state[1]-track.formery)<0.00001):
                track.counter=track.counter+1
            else:
                track.counter = 0
            if track.counter==10:
                tracks_to_delete.append(track)
                self.reason_id.append(track.track_id)
                self.reason.append(1)
                # df = pn.DataFrame({'track_id': self.reason_id, 'reason' :self.reason})
                # # csv_file = 'C:\\Users\\danie\\OneDrive\\שולחן העבודה\\scoter\\recording_scooter\\track_log.csv'
                # df.to_csv(csv_file, index=False)

                # print('hel')


            track.formerx = track.state[0]
            track.formery = track.state[1]


            # print('state',track.state)
            # print('meas',track.measurement)
            # 2
            if abs(track.state[1]-track.measurement[1])>7:
                track.counter2 = track.counter2 +1
            else:
                track.counter2=0
            #     15
            if track.counter2==30:
                tracks_to_delete.append(track)
                self.reason_id.append(track.track_id)
                self.reason.append(2)
                # df = pn.DataFrame({'track_id': self.reason_id, 'reason' :self.reason})
                # csv_file = 'C:\\Users\\danie\\OneDrive\\שולחן העבודה\\scoter\\recording_scooter\\track_log.csv'
                # df.to_csv(csv_file, index=False)
                # print('hel')
        for i, t1 in enumerate(self.tracks):
            aa = t1.state[1]
            for t2 in self.tracks[i + 1:]:
                aa2 = t2.state[1]
                if abs(aa2 - aa) < 4 and abs(aa2 - aa) != 0:
                    t1.counter3 += 1
                    # print('counter3', t2.counter3)
                    # print('counter3', t2.track_id)
                    # c=6
                else:
                    t1.counter3 = 0
                    # print('counter3', t2.counter3)
                    # print('counter3', t2.track_id)

        for track in self.tracks:
            if track.counter3==5:
                tracks_to_delete.append(track)
                self.reason_id.append(track.track_id)
                self.reason.append(3)
                # df = pn.DataFrame({'track_id': self.reason_id, 'reason' :self.reason})
                # csv_file = 'C:\\Users\\danie\\OneDrive\\שולחן העבודה\\scoter\\recording_scooter\\track_log.csv'
                # df.to_csv(csv_file, index=False)


        for track in self.tracks:
                 if abs(track.measurement[1] - track.prev_meas)<0.00001:
                     track.counter4 = track.counter4 + 1
                 else:
                     track.counter4 = 0
                 track.prev_meas = track.measurement[1]

        for track in self.tracks:
                 if track.counter4 == 10:
                 # if track.counter4 == 50:
                     tracks_to_delete.append(track)
                     self.reason_id.append(track.track_id)
                     self.reason.append(4)
                     # df = pn.DataFrame({'track_id': self.reason_id, 'reason': self.reason})
                     # csv_file = 'C:\\Users\\danie\\OneDrive\\שולחן העבודה\\scoter\\recording_scooter\\track_log.csv'
                     # df.to_csv(csv_file, index=False)
                     # print('id',track.track_id)

        for track in self.tracks:
                 if abs(track.state[1] - track.prev_state)<0.000001:
                     track.counter5 = track.counter5 + 1
                 else:
                     track.counter5 = 0
                 track.prev_state = track.state[1]

        for track in self.tracks:
                 if track.counter5 == 10:
                     tracks_to_delete.append(track)
                     self.reason_id.append(track.track_id)
                     self.reason.append(5)
                     # df = pn.DataFrame({'track_id': self.reason_id, 'reason': self.reason})
                     # csv_file = 'C:\\Users\\danie\\OneDrive\\שולחן העבודה\\scoter\\recording_scooter\\track_log.csv'
                     # df.to_csv(csv_file, index=False)
                     # print('id',track.track_id)

        for track in self.tracks:
                 # print('track.first_state',track.first_state)
                 if abs(track.state[1]-track.first_state)<0.55:
                     track.counter6=track.counter6+1
                 else:
                     track.counter6=0


        for track in self.tracks:
                 if track.counter6 == 100:
                     tracks_to_delete.append(track)
                     self.reason_id.append(track.track_id)
                     self.reason.append(6)
                     # df = pn.DataFrame({'track_id': self.reason_id, 'reason' :self.reason})
                     # csv_file = 'C:\\Users\\danie\\OneDrive\\שולחן העבודה\\scoter\\recording_scooter\\track_log.csv'
                     # df.to_csv(csv_file, index=False)


        for track_to_delete in tracks_to_delete:
             # print('del',track_to_delete.track_id)
             if track_to_delete in self.tracks:
                self.tracks.remove(track_to_delete)


class Track:
    def __init__(self, track_id, initial_measurement):
        self.track_id = track_id  # Unique ID for the track
        self.measurement = initial_measurement  # Initial measurement [x, y]
        self.state = np.array([initial_measurement[0], initial_measurement[1], 0, 0])  # State [x, y, vx, vy]
        self.age = 0  # Age of the track
        self.range_history = []
        self.state_history = [self.state.copy()]
        self.formerx=0
        self.formery=0
        self.counter=0
        self.counter2 = 0
        self.counter3 =0
        self.counter4=0
        self.counter5 = 0
        self.prev_meas=0
        self.prev_state= 0
        self.first_state=0
        self.counter6=0

    def update(self, new_measurement):
        # Update the state and measurement
        self.state = np.array([new_measurement[0], new_measurement[1], 0, 0])
        self.measurement = new_measurement
        self.age += 1
        self.range_history.append(new_measurement[1])
        # self.state_history.append(self.state.copy())



# import scipy.io
# file_path = r'C:\Users\\danie\\OneDrive\\שולחן העבודה\\נווה אור\\new 28_3\\py.mat'
# # file_path = r'C:\Users\\danie\\OneDrive\\שולחן העבודה\\נווה אור\\new 28_3\\py_newa9.mat'
# mat = scipy.io.loadmat(file_path)
# dis = mat['dist2']
# # dis = mat['distance']
#
# id=[]
# range=[]
# def main():
#     tracker = Tracker(merge_threshold=3)  # Set merge_threshold to 3
#     k = 0
#     while True:
#         if k==3430:
#         # if k ==9500:
#         # if k == 1100:
#             df = pn.DataFrame({'id': id, 'range': range})
#             # csv_file = 'C:\\Users\\danie\\OneDrive\\שולחן העבודה\\scoter\\recording_scooter\\id9.csv'
#             df.to_csv(csv_file, index=False)
#             break
#



        k = k + 1
        x = 0
        y = dis[0, k]  # Using 'dis' from your previous code
        if y>4:
            measurements = [[x, y]]
            tracker.update(measurements)

            for track in tracker.tracks:
                print("Track ID:", track.track_id, "State:", track.state, "Age:", track.age)
                id.append(track.track_id)
                range.append(track.state[1])

            # time.sleep(0.01)  # Simulate a delay, replace with your actual radar update rate
            print('Number of tracks:', len(tracker.tracks))

# print(range)





if __name__ == "__main__":
    main()