#!/usr/bin/env python3
# Recognizes macro-scale patterns in pointcloud data.
# Confined to rectangles (for now)

# Instead of using lines, try storing points and connections (a graph, effectively)
# Connection includes: point, confidence (<50% means more likely no connection, >50% means more likely closed)
