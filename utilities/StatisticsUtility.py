from math import sqrt


def calculateMean(data):
    if len(data) > 0:
        summedValues = 0
        for dataPoint in data:
            summedValues += dataPoint

        return summedValues / len(data)
    else:
        return 0


class MovingAverage:
    def __init__(self, length):
        self.__data = []
        self.__length = length

    def addDataPoint(self, dataPoint):
        self.__data.append(dataPoint)

    def addData(self, data):
        self.__data.extend(data)

    def calculateAverage(self):
        self._trimData()
        return calculateMean(self.__data)

    def addDataPointAndCalculateAverage(self, dataPoint):
        self.addDataPoint(dataPoint)
        return self.calculateAverage()

    def addDataAndCalculateAverage(self, data):
        self.addData(data)
        return self.calculateAverage()

    def _trimData(self):
        while len(self.__data) > self.__length:
            self.__data.pop(0)


class MovingStandardDeviation:
    def __init__(self, length):
        self.__data = []
        self.__length = length

    def addDataPoint(self, dataPoint):
        self.__data.append(dataPoint)

    def addData(self, data):
        self.__data.extend(data)

    def calculateStandardDeviation(self):
        self._trimData()
        mean = calculateMean(self.__data)

        summedValues = 0
        for dataPoint in self.__data:
            summedValues += (dataPoint - mean) ** 2

        return sqrt(summedValues / len(self.__data))

    def addDataPointAndCalculateStandardDeviation(self, dataPoint):
        self.addDataPoint(dataPoint)
        return self.calculateStandardDeviation()

    def addDataAndCalculateStandardDeviation(self, data):
        self.addData(data)
        return self.calculateStandardDeviation()

    def _trimData(self):
        while len(self.__data) > self.__length:
            self.__data.pop(0)
