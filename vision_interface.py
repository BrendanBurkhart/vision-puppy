from networktables import NetworkTables


class VisionTracking:
    def __init__(self):
        self.table = NetworkTables.getTable("vision")

    def cube_position(self):
        visible = self.table.getString("orientation", "") != ""
        direction = self.table.getNumber("xdir", None)
        offset = self.table.getNumber("xmag", None)

        if not visible or not offset:
            return None

        return offset, direction
