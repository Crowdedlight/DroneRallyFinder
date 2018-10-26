import affine
import gdal
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import rasterio
from scipy.ndimage.measurements import label
from scipy import ndimage
from math import ceil, hypot
from gdal import osr
import geopy.distance
import cv2

class RallyPoint:
    def __init__(self):
        # variables for class
        self.px = 0
        self.py = 0
        self.utm_x = None
        self.utm_y = None
        self.lat = None
        self.lon = None
        self.size = 0
        self.distance = 0

    # override to_string functions to get debug print that is actually useful
    def __str__(self):
        return f'[px: {self.px}, py: {self.py}, utm_x: {self.utm_x}, utm_y: {self.utm_y}, lat: {self.lat}, lon: {self.lon}, size: {self.size}, distance: {self.distance}]'
        # return f'[size: {self.size}, distance: {self.distance}]'

    def __repr__(self):
       return (f'{self.__class__.__name__}('
               f'[px: {self.px}, py: {self.py}, utm_x: {self.utm_x!r}, utm_y: {self.utm_y!r}, lat: {self.lat!r}, lon: {self.lon!r}, size: {self.size!r}, distance: {self.distance!r}]')
               # f'[size: {self.size!r}, distance: {self.distance!r}]')


class LatLon:
    def __init__(self):
        self.lat = None
        self.lon = None

    # override to_string functions to get debug print that is actually useful
    def __str__(self):
        return f'[lat: {self.lat}, lon: {self.lon}]'

    def __repr__(self):
       return (f'{self.__class__.__name__}('
               f'[lat: {self.lat!r}, lon: {self.lon!r}]')


class DroneRallypointFinder:

    def __init__(self):

        self.src = None
        self.slope = None
        self.flat_areas = None
        self.labeled = None

        # drone properties
        self.curr_pos = LatLon()
        self.drone_max_range = 0

        # Image properties
        self.xoff = 0
        self.xres = 0
        self.xskew = 0
        self.yoff = 0
        self.yskew = 0
        self.yres = 0


    def calcMaxRallyCoordinates(self, curr_pos, max_distance):
        # current pos in UTM x,y
        # calculate circle with base in curr_pos

        None

    def calcSlope(self, DEM):
        # calculate DEM and save in tmp file
        gdal_options = gdal.DEMProcessingOptions(computeEdges=True, band=1, slopeFormat="degree")
        gdal.DEMProcessing('tmp_slope.tif', DEM, 'slope', options=gdal_options)

        # open tmp file and read DEM and return
        with rasterio.open('tmp_slope.tif') as dataset:
            slope = dataset.read(1)
        return slope

    def findPossibleRallyPoints(self, slope, degrees, drone_size):
        # copy slope and mark everything under degrees as 255 => black
        flat_mask = slope <= degrees

        # Copy slope for same dimensions, and fill all values with white
        flat_areas = np.copy(slope)
        flat_areas[:, :] = 0

        # mark flat enough surfaces as 1 (Working binary now)
        flat_areas[flat_mask] = 1

        # drone size, needs to be converted to pixel values. 0.4m per px
        drone_size_px = drone_size / 0.4
        # erode and dilate with drone-size to only keep places that is big enough for drone to land
        struct_8_connected = ndimage.generate_binary_structure(2, 2)

        flat_mask = ndimage.binary_opening(flat_areas, struct_8_connected, ceil(drone_size_px))

        # reset flat areas and mark all regions after filtering
        flat_areas[:, :] = 0
        flat_areas[flat_mask] = 1

        # save for debugging in other functions
        self.flat_areas = np.copy(flat_areas)

        # do connected components to find all groups. Used to sort for best suitability besides distance
        # this defines the connection filter
        structure = np.ones((3, 3), dtype=np.int)
        labeled, ncomponents = label(flat_areas, structure)

        # save for debugging in other functions
        self.labeled = np.copy(labeled)

        # add distance from drone to center-of-mass position on every labeled group, together with size of group
        # x,y distance from drone pos to center pixel. Every pixel is 0,4m.
        lbl = ndimage.label(labeled)[0]
        center_of_mass_lbl = ndimage.measurements.center_of_mass(labeled, labels=lbl, index=range(1, ncomponents+1))

        # calc surface areas of label regions
        surface_areas = np.bincount(labeled.flat)[1:]

        rally_points = []

        # now we got list of centroids for all regions and their size. Calc utm coordinates for each centroid now
        # make list of regions, with their centroid, centroid UTM, size
        for idx, point in enumerate(center_of_mass_lbl):
            # get utm and lat/lon coords
            x_utm, y_utm = self.convertPixel2UTM(point[1], point[0])
            lat, lon = self.convertUTM2WGS84(x_utm, y_utm)

            # make object
            rally_point = RallyPoint()
            # Function returns Y,X format.
            rally_point.px = point[1]
            rally_point.py = point[0]
            rally_point.utm_x = x_utm
            rally_point.utm_y = y_utm
            rally_point.lat = lat
            rally_point.lon = lon
            rally_point.size = surface_areas[idx]

            rally_points.append(rally_point)

        # return list for sorting based on distance from drone
        return rally_points

    def sortRallyPoints(self, rally_points):
        # sort based on distance from current pos, and size. Shortest distance and biggest size first
        for point in rally_points:
            # get distance and add to list
            dist = self.dist2pointUTM(point.utm_x, point.utm_y)
            point.distance = dist

        # sort based on distance and size TODO, change to cost functions.
        # TODO Calculate cost for each point based on better weight of size versus distance, then sort based on cost
        rally_points_sorted = sorted(rally_points, key=lambda x: (x.distance, x.size))
        return rally_points_sorted

    def dist2point(self, lat, lon):
        # calculates distance from drone curr_pos to rally_point. All input should be lat/lon
        coords_1 = (lat, lon)
        coords_2 = (self.curr_pos.lat, self.curr_pos.lon)

        # calculate distance using WGS 84
        dist = geopy.distance.vincenty(coords_1, coords_2).meters
        return dist

    def dist2pointUTM(self, x, y):
        d_x, d_y = self.convertLatLon2UTM(self.curr_pos.lat, self.curr_pos.lon)
        dist = hypot(x - d_x, y - d_y)
        return dist

    def convertPixel2UTM(self, pixel_x, pixel_y):
        xp = self.xres * pixel_x + self.xskew * pixel_y + self.xoff
        yp = self.yskew * pixel_x + self.yres * pixel_y + self.yoff
        return xp, yp

    def convertUTM2pixel(self, x, y):
        # solution found on stackoverflow, that also takes into account if image is skewed
        """Return floating-point value that corresponds to given point."""
        forward_transform = \
            affine.Affine.from_gdal(*self.src.GetGeoTransform())
        reverse_transform = ~forward_transform
        px, py = reverse_transform * (x, y)
        px, py = int(px + 0.5), int(py + 0.5)
        pixel_coord = px, py

        return pixel_coord[0], pixel_coord[1]

    def convertLatLon2UTM(self, lat, lon):
        # get spatiel reference and import from raster
        srs = osr.SpatialReference()
        srs.ImportFromWkt(self.src.GetProjection())

        srsLatLong = srs.CloneGeogCS()
        # change target and source around, based on what way from WGS84 --> UTM you want to go
        ct = osr.CoordinateTransformation(srsLatLong, srs)
        (x, y, _) = ct.TransformPoint(lon, lat)

        return x, y

    def convertUTM2WGS84(self, x, y):
        # get spatiel reference and import from raster
        srs = osr.SpatialReference()
        srs.ImportFromWkt(self.src.GetProjection())

        srsLatLong = srs.CloneGeogCS()
        # change target and source around, based on what way from WGS84 --> UTM you want to go
        ct = osr.CoordinateTransformation(srs, srsLatLong)
        (lon, lat, _) = ct.TransformPoint(x, y)
        return lat, lon

    def showImg(self, img, cmap=None):
        plt.figure(figsize=(6, 4))
        if cmap is None:
            plt.imshow(img)
        elif cmap is not None:
            plt.imshow(img, cmap=cmap)
        plt.show()

    def showImgCompared(self, img1, img2):
        plt.figure(figsize=(9, 3))
        ax1 = plt.subplot(1, 2, 1)
        ax1.imshow(img1, cmap="Greys")
        ax2 = plt.subplot(1, 2, 2, sharex=ax1, sharey=ax1)
        ax2.imshow(img2, cmap="Greys")
        plt.show()

    def showSortedRallyPoints(self, points):
        # take map, colour drone pos; colour rest based on order in list
        # copy shape
        w, h = self.slope.shape

        # convert to color
        map_points = np.empty((w, h, 3), np.uint8)
        map_points[:, :, 0] = self.slope
        map_points[:, :, 1] = self.slope
        map_points[:, :, 2] = self.slope

        # reverse the colours
        map_points = ~map_points

        # get drone pos in pixels require having it in UTM
        x, y = self.convertLatLon2UTM(self.curr_pos.lat, self.curr_pos.lon)
        drone_px, drone_py = self.convertUTM2pixel(x, y)

        # Show original slope with the zones drawed as 10x10 squares
        scale = 10
        colour = (0, 240, 0)
        # drone is marked red
        cv2.circle(map_points, (drone_px, drone_py), 10, (0, 0, 240), 3)

        for idx, point in enumerate(points):
            cn1 = (int(point.px-scale), int(point.py-scale))
            cn2 = (int(point.px+scale), int(point.py+scale))
            # opencv rectangle wants corners in format: (Y,X). VERY IMPORTANT
            cv2.rectangle(map_points, cn1, cn2, colour, 3)

            # put number inside rect at px, py coord. Set text options
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            font_thick = 1
            text = str(idx)

            # calculate text size
            textSize = cv2.getTextSize(text, font, font_scale, font_thick)
            txt_half_x = int(textSize[0][0] / 2)
            txt_half_y = int(textSize[0][1] / 2)
            # center text
            textPos = (int(point.px) - txt_half_x, int(point.py) + txt_half_y)

            # place text
            cv2.putText(map_points, text, textPos, font, font_scale, (0, 0, 0), font_thick)


        # self.showImg(map_points)
        # self.showImgCompared(map_points, self.flat_areas)
        self.showImgCompared(map_points, self.labeled)

    def run(self):

        # Every pixel is 0,4m. So all 2500 pixels in one direction corresponds to 1000m in that direction
        self.curr_pos.lat = 55.36630521167112
        self.curr_pos.lon = 10.432722944095936
        self.drone_max_range = 2000  # meters

        # read in image
        self.src = gdal.Open('DHM/DSM_1km_6136_590.tif')
        # GDAL affine transform parameters, According to gdal documentation xoff/yoff are image left corner,
        # xres/yres are pixel wight/height and xskew/yskew is rotation and is zero if image is north up.
        self.xoff, self.xres, self.xskew, self.yoff, self.yskew, self.yres = self.src.GetGeoTransform()

        # debug
        self.slope = self.calcSlope(self.src)

        print(self.slope.shape)
        print(np.min(self.slope))
        print(np.max(self.slope))
        print("##### Test conversion lat/lon -> UTM -> Pixel -> UTM works #####")
        utm_x, utm_y = self.convertLatLon2UTM(self.curr_pos.lat, self.curr_pos.lon)
        d_px, d_py = self.convertUTM2pixel(utm_x, utm_y)
        print(utm_x, utm_y)
        print(d_px, d_py)
        print(self.convertPixel2UTM(d_px, d_py))
        print("#############")

        # max slope of 2 degrees, and area has to be 5x5m for safety
        possible_spots = self.findPossibleRallyPoints(self.slope, 2, 5)
        rally_points_sorted = self.sortRallyPoints(possible_spots)

        # show results
        self.showSortedRallyPoints(rally_points_sorted)



        # Calculate distance drone can go with current emergency, before having to land

        # Obtain min and max coordinates in UTM to make a area the drone can search in

        # Get tiles covering this area

        # merge tiles to have a complete map? Then mask out everything but the circle the drone can operate in. Mask with black, as slope returns low slopes as white

        # convert tiles to slope degrees with gdal.

        # TODO maybe sort all "water"/roads out, as their slope often will be close to zero too. For now this is what the image validator will do

        # find all areas where slope is less than x degrees, and the connected area is bigger than y, to indicate the land is flat enough for the drone to land safely

        # rank the list over found areas on suitability, like distance from drone, and how big the area is flat in

        # output list with coordinates, possible images, over best areas




if __name__ == '__main__':
    DroneRallypointFinder().run()
