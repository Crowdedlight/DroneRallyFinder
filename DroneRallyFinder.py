import gdal
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import rasterio
from scipy.ndimage.measurements import label
from scipy import ndimage
from math import floor, ceil



class DroneRallypointFinder():

    def __init__(self):
        None


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

        # do connected components to find all groups. Used to sort for best suitability besides distance
        # this defines the connection filter
        structure = np.ones((3, 3), dtype=np.int)
        labeled, ncomponents = label(flat_areas, structure)

        # add distance from drone to center-of-mass position on every labeled group, together with size of group
        # x,y distance from drone pos to center pixel. Every pixel is 0,4m.
        lbl = ndimage.label(labeled)[0]
        center_of_mass_lbl = ndimage.measurements.center_of_mass(labeled, labels=lbl, index=range(1, ncomponents+1))

        # calc surface areas of label regions
        surface_areas = np.bincount(labeled.flat)[1:]

        print(center_of_mass_lbl)
        print(surface_areas)

        # now we got list of centroids for all regions and their size. Calc utm coordinates for each centroid now

        # make list of regions, with their centroid, centroid UTM, size

        # return list for sorting based on distance from drone

    def showImg(self, img, cmap=None):
        plt.figure(figsize=(6, 4))
        if cmap is None:
            plt.imshow(img)
        elif cmap is not None:
            plt.imshow(img, cmap=cmap)
        plt.show()

    def run(self):

        # Every pixel is 0,4m. So all 2500 pixels in one direction corresponds to 1000m in that direction

        # debug
        slope = self.calcSlope('DHM/DSM_1km_6136_590.tif')

        print(type(slope))
        print(slope.dtype)
        print(slope.shape)
        print(np.min(slope))
        print(np.max(slope))

        # max slope of 2 degrees, and area has to be 5x5m for safety
        possible_spots = self.findPossibleRallyPoints(slope, 2, 5)

        None

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
