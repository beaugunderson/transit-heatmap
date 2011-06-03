#!/usr/bin/env python2.7

from __future__ import division

import json
import sys
import time

from datetime import datetime, timedelta
from math import radians, cos, sin, atan2, sqrt

#from IPython.Shell import IPShellEmbed
#ipshell = IPShellEmbed()

from collections import namedtuple

from graphserver.ext.routeserver.events import BoardEvent, DescribeCrossingAtAlightEvent, AlightEvent, StreetEvent, HeadwayBoardEvent, HeadwayAlightEvent
from graphserver.ext.routeserver.events import StreetTurnEvent, StreetStartEvent, StreetEndEvent
from graphserver.ext.osm.osmdb import OSMDB

from graphserver.core import Graph, Link, Street, State, WalkOptions, Combination
from graphserver.graphdb import GraphDatabase

timezone = 'America/Los_Angeles'
gtfsdb = 'king.gtfsdb'
osmdb = 'king.highway.osmdb'

NearbyVertex = namedtuple('NearbyVertex', 'id latitude longitude distance_miles')

edge_events = []
vertex_events = []

for e in [BoardEvent, DescribeCrossingAtAlightEvent, AlightEvent,
        HeadwayBoardEvent, HeadwayAlightEvent]:
    edge_events.append(e(gtfsdb_filename=gtfsdb, timezone_name=timezone))

edge_events.append(StreetEvent(osmdb_filename=osmdb, timezone_name=timezone))

for e in [StreetTurnEvent, StreetStartEvent, StreetEndEvent]:
    vertex_events.append(e(osmdb_filename=osmdb, timezone_name=timezone))

class ModifiedOSMReverseGeocoder:
    def __init__(self, osmdb_filename, range=0.005):
        self.osmdb = OSMDB(osmdb_filename)
        self.range = range

    def __call__(self, lat, lon):
        #vertex = self.nearest_node(lat, lon, use_index=False, range=self.range)
        vertex = self.nearest_node(lat, lon, use_index=True, range=self.range)

        if vertex:
            return NearbyVertex(*vertex)
        else:
            return None

    def bounds(self):
        """return tuple representing bounding box of reverse geocoder with form 
        # (left, bottom, right, top)"""
        return self.osmdb.bounds()

    def nearest_node(self, lat, lon, use_index=True, range=0.005):
        c = self.osmdb.get_cursor()

        if use_index and self.osmdb.index:
            id = list(self.osmdb.index.nearest((lon, lat), 1))[0]

            c.execute("SELECT id, lat, lon FROM nodes WHERE id = ?", (id,))
        else:
            c.execute("SELECT id, lat, lon FROM nodes WHERE " +
                "endnode_refs > 1 AND lat > ? AND lat < ? AND lon > ? AND lon < ?",
                 (lat-range, lat+range, lon-range, lon+range))

        dists = [(nid, nlat, nlon, self.haversine_miles(nlon, nlat, lon, lat)) for nid, nlat, nlon in c]

        if not dists:
            return (None, None, None, None)

        return min(dists, key=lambda x:x[3])

    def haversine_miles(self, lon1, lat1, lon2, lat2):
        """Calculate the great circle distance between two points
        on the earth (specified in decimal degrees)"""

        # convert decimal degrees to radians
        lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
        # haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1

        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2

        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        miles = 3963.0 * c

        return miles

class SelfEncoderHelper(json.JSONEncoder):
    def default(self, obj):
        if hasattr(obj, "to_jsonable"):
            return obj.to_jsonable()

        return json.JSONEncoder.default(self, obj)

def postprocess_path_raw(vertices, edges):
    retbuilder = []

    retbuilder.append("vertices & states")

    for i, vertex in enumerate(vertices):
        if not i:
            ipshell()

        retbuilder.append("%d %s" % (i, str(vertex)))
        retbuilder.append("%d %s" % (i, str(vertex.state)))

    retbuilder.append("")
    retbuilder.append("edges")

    for i, edge in enumerate(edges):
        if not i:
            ipshell()

        retbuilder.append("%d %s" % (i, str(edge.payload)))

    return "\n".join(retbuilder)

def postprocess_path(vertices, edges):
    context = {}

    for edge1, vertex1, edge2, vertex2 in zip([None] + edges, vertices, edges + [None], vertices[1:] + [None, None]):
        for handler in vertex_events:
            if handler.applies_to(edge1, vertex1, edge2):
                event = handler(edge1, vertex1, edge2, context=context)

                if event is not None:
                    yield handler.__class__.__name__, event

        for handler in edge_events:
            if handler.applies_to(vertex1, edge2, vertex2):
                event = handler(vertex1, edge2, vertex2, context=context)

                if event is not None:
                    yield handler.__class__.__name__, event

def path(lat, lon, destination, graph, walk_options, reverse_geocode,
        current_time=None):
    origin = reverse_geocode(lat, lon)

    if not origin or not origin.id: # or \
        #    origin.distance_miles >= 0.5:
        return (None, None)

    origin_walk_penalty = timedelta(minutes=(origin.distance_miles / walk_options.walking_speed) * 60)

    if current_time is None:
        current_time = int(time.time())

    start_time = None
    stop_time = None

    tree = None

    try:
        tree = graph.shortest_path_tree('osm-%s' % origin.id, 'osm-%s' % destination.id, State(1, current_time), walk_options)

        vertices, edges = tree.path('osm-%s' % destination.id)

        start = None
        stop = None

        for v in vertices:
            if v.state.time:
                if not start:
                    start = v.state.time

                stop = v.state.time

        start_time = datetime.fromtimestamp(start)
        stop_time = datetime.fromtimestamp(stop)

        start_time -= origin_walk_penalty
    finally:
        if tree:
            tree.destroy()

        return (start_time, stop_time)

    #narrative = list(postprocess_path(vertices, edges))
    #return json.dumps({ 'narrative': narrative }, indent=3, cls=SelfEncoderHelper)

walk_options = WalkOptions()

walk_options.transfer_penalty = 0
walk_options.walking_speed = 1.0
walk_options.hill_reluctance = 1.5
#walk_options.turn_penalty = 
#walk_options.walking_reluctance = 
#walk_options.max_walk = 

# The full Metro King County boundaries
#lon_start = -122.506729
#lon_stop = -121.785828
#lat_start = 47.9323654
#lat_stop = 47.1891136

# Seattle & the East side
lat_start = 47.7
lat_stop = 47.5
lon_start = -122.433
lon_stop = -122.04

dt = datetime(2011, 2, 22, 9, 0, 0)
current_time = time.mktime(dt.timetuple())

max_density = 100

lat_distance = abs(lat_stop - lat_start)
lon_distance = abs(lon_stop - lon_start)

# Normalize a square density for a possibly rectangular grid
if lat_distance > lon_distance:
    lat_grid = max_density
    lon_grid = int((lon_distance / lat_distance) * max_density)
else:
    lon_grid = max_density
    lat_grid = int((lat_distance / lon_distance) * max_density)

lat_increment = (lat_stop - lat_start) / lat_grid
lon_increment = (lon_stop - lon_start) / lon_grid

geocoder_range = abs(min((lat_increment / 3, lon_increment / 3)))

reverse_geocode = ModifiedOSMReverseGeocoder(osmdb_filename=osmdb,
        range=geocoder_range)

# My office
destination_lat = 47.611512
destination_lon = -122.334329

destination = reverse_geocode(destination_lat, destination_lon)

print "Point density", max_density
print "Using grids lat %d, lon %d" % (lat_grid, lon_grid)
print "Increments lat %f, lon %f" % (lat_increment, lon_increment)
print "Geocoder range", geocoder_range
print "Using time", dt.ctime()
print "Bounding box tlbr: %f, %f, %f, %f" % (lat_start, lon_start, lat_stop, lon_stop)
print "Checking %d routes" % (lat_grid * lon_grid)

minimum = sys.maxint
maximum = -sys.maxint - 1

min_lon = sys.maxint
max_lon = -sys.maxint - 1

min_lat = sys.maxint
max_lat = -sys.maxint - 1

gdb = GraphDatabase('king.highway.gdb')
graph = gdb.incarnate()

rows = []

#missing_value = None
missing_value = 50000

for i in xrange(0, lat_grid):
    row = []

    print "Row %d" % i

    lat = lat_start + (i * lat_increment)

    if lat < min_lat:
        min_lat = lat

    if lat > max_lat:
        max_lat = lat

    for j in xrange(0, lon_grid):
        lon = lon_start + (j * lon_increment)

        if lon < min_lon:
            min_lon = lon

        if lon > max_lon:
            max_lon = lon

        # Provide padding
        if i == 0 or i == lat_grid - 1 or \
                j == 0 or j == lon_grid - 1:
            row.append(missing_value)

            continue

        start, stop = path(lat, lon, destination, graph,
                walk_options, reverse_geocode=reverse_geocode, current_time=current_time)

        if not start or not stop:
            row.append(missing_value)

            continue

        delta = stop - start

        trip_time = delta.total_seconds()

        if trip_time < minimum:
            minimum = trip_time

        if trip_time > maximum:
            maximum = trip_time

        row.append(trip_time)

    rows.append(row)

with open('output-map.txt', 'w') as f:
    for i in rows:
        for j in i:
            if j:
                f.write("X")
            else:
                f.write(" ")

        f.write("\n")

with open('output.js', 'w') as f:
    f.write("var output_normal = %s;\n" % (json.dumps({
        'min_z': minimum,
        'max_z': maximum,
        'min_lon': min_lon,
        'max_lon': max_lon,
        'min_lat': min_lat,
        'max_lat': max_lat,
        'data': rows}, indent=3)))
