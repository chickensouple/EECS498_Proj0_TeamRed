

class Directions:
	PosX = 0
	NegX = 1
	PosY = 2
	NegY = 3

	Names = ["PosX", "NegX", "PosY", "NegY"]

	@staticmethod
	def opposite(direction):
		if (direction == DirectionsPosX):
			return Directions.NegX
		if (direction == DirectionsNegX):
			return Directions.PosX
		if (direction == DirectionsPosY):
			return Directions.NegY
		if (direction == DirectionsNegY):
			return Directions.PosY
	