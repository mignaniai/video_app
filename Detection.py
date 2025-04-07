import msgpack


class Detection:
    def __init__(self, jsonText=None):
        if jsonText is None:
            pass
        else:
            self.id = jsonText["id"]
            self.posXmeter = jsonText["posXmeter"]
            self.posYmeter = jsonText["posYmeter"]
            self.velXm2sec = jsonText["velXm2sec"]
            self.velYm2sec = jsonText["velYm2sec"]
            self.range = jsonText["range"]
            self.bearing = jsonText["bearing"]
            self.classification = jsonText["classification"]

    def setParameters(self, classification,id, posX, posY, velX, velY, range, bearing):
        self.id = id
        self.posXmeter = posX
        self.posYmeter = posY
        self.velXm2sec = velX
        self.velYm2sec = velY
        self.range = range
        self.bearing = bearing
        self.classification = classification

    def setId(self, id):
        self.id = id

    def to_dict(self):
        return {'id': self.id,
                'posXmeter': self.posXmeter, 'posYmeter': self.posYmeter,
                'velXm2sec' : self.velXm2sec,'velYm2sec' : self.velYm2sec,
                'range': self.range,'bearing':self.bearing,'classification':self.classification}


class TrackerUI:
    def __init__(self,data=None):
        if data is None:
            self.detections = []
            self.radarStatus = 0
            self.batteryPower = 0
            self.isFiltered = 0
        else:
            detect = data["detections"]
            self.detections = detect
            self.radarStatus = data["radarStatus"]
            self.batteryPower = data["batteryPower"]
            self.isFiltered = data["isFiltered"]


    def to_dict(self):
        return {'detections': [detection.to_dict() for detection in self.detections],
                'radarStatus':self.radarStatus,
                'batteryPower':self.batteryPower,
                'isFiltered':self.isFiltered}

class Track:
    def __init__(self):
        self.trackId=0
        self.age = 0
        self.reliability = 0
        self.classification = 1
        self.posX = 0
        self.posY = 0
        self.posZ = 0
        self.velX = 0
        self.velY = 0
        self.velZ = 0
        self.time_stamp = 0


    def setParameters(self, id, posX, posY, velX, velY):
        self.trackId=id
        self.age = 0
        self.reliability = 2
        self.posX = float(posX)
        # self.posX = 4
        self.posY = float(posY)
        # self.posY = 5
        self.posZ = 0
        self.velX = float(velX)
        # self.velX = 7
        self.velY = float(velY)
        # self.velY = 8
        self.velZ = 0
        self.time_stamp = 0

    def to_list(self):
        return [
            self.trackId,
            self.age,
            self.reliability,
            self.classification,
            self.posX,
            self.posY,
            self.posZ,
            self.velX,
            self.velY,
            self.velZ,
            self.time_stamp
        ]

class Tracks:
    def __init__(self, ):
        self.tracks=[]
        self.frameNumber = 0
        self.numOfTracks = 0
        self.rangeCoverage = 0
        self.detectionsNoise = 0

    def set_length(self, length):
        self.numOfTracks = length

    def add_radar_data(self, radar_track):
        self.tracks.append(radar_track)

    def to_list(self):
        return [
            [track1.to_list() for track1 in self.tracks],
            self.frameNumber,
            self.numOfTracks,
            self.rangeCoverage,
            self.detectionsNoise
        ]

    def to_binary(self):
        packed_data = msgpack.packb(
            [
                [track1.to_dict() for track1 in self.tracks],
                self.frameNumber,
                self.numOfTracks,
                self.rangeCoverage,
                self.detectionsNoise
            ],
            use_bin_type=True
        )
        return packed_data

   # # Serialize to MsgPack
   #  def to_msgpack(self):
   #      return msgpack.packb(self.to_list(), use_bin_type=False)
