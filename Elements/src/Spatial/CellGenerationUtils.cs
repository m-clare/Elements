
using System.Collections.Generic;
using System.Linq;
using Elements.Geometry;
using NetTopologySuite.Operation.Polygonize;
using NetTopologySuite.Algorithm;
using ng = NetTopologySuite.Geometries;
using Newtonsoft.Json;
using System;

namespace Elements.Spatial
{
    /// <summary>
    /// This class contains methods useful for dividing a profile/polygon into
    /// smaller pieces using arbitrary other geometry, i.e. Lines.
    /// Internally it uses the NetTopologySuite library https://github.com/NetTopologySuite/NetTopologySuite.
    /// </summary>
    public static class CellGenerationUtils
    {
        /// <summary>
        /// Given a profile and a list of lines, return the individual polygons the result from
        /// dividing that profile along all of the lines.
        /// </summary>
        public static IEnumerable<Polygon> Polygonize(List<Line> lines, Profile workingArea)
        {
            var linesExtended = lines.Select(s => s.Extend(0.01)); // this is a hack but it works!
            var lineStrings = linesExtended.Select(l => l.ToLineString()).ToArray();
            ng.Geometry nodedLineStrings = lineStrings[0];
            for (int i = 1; i < lineStrings.Count(); i++)
            {
                try
                {
                    nodedLineStrings = nodedLineStrings.Union(lineStrings[i]);
                }
                catch
                {
                    //TODO - figure out why - maybe overlaps?
                }
            }
            var polygonizer = new Polygonizer();

            for (int i = 0; i < nodedLineStrings.NumGeometries; i++)
            {
                var linestring = nodedLineStrings.GetGeometryN(i) as ng.LineString;
                polygonizer.Add(linestring);
            }

            // there is a chance for odd shaped cells that the centroid might fall outside of its boundary
            // TODO: use a better method than the centroid to check inclusion
            var polygonsRaw = polygonizer.GetPolygons().Where(p => IsWithinProfile(p.Centroid.Coordinate.ToVector3(), workingArea));


            var polygons = polygonsRaw.Select(p => p.ToPolygon());
            return polygons;
        }

        public static List<Line> TrimCenterlinesWithBoundaries(Profile profile, IEnumerable<Line> linesToTrim)
        {
            List<Line> trimmedLines = new List<Line>();

            foreach (var line in linesToTrim)
            {
                var polygon = profile.Perimeter;
                var trimmed = line.Trim(polygon, out var trimmedOut);
                trimmedLines.AddRange(trimmed);
            }
            if (profile.Voids != null && profile.Voids.Count > 0)
            {
                for (int i = 0; i < profile.Voids.Count; i++)
                {
                    var tempLines = new List<Line>();
                    var polygon = profile.Voids[i];
                    foreach (var line in trimmedLines)
                    {
                        var trimmed = line.Trim(polygon, out var trimmedOut);
                        tempLines.AddRange(trimmedOut);
                    }
                    trimmedLines = tempLines;
                }
            }

            return trimmedLines;
        }

        private struct PointList
        {
            public int index;
            public List<Vector3> Vertices;
            public string discriminator;
        }

        public static List<Line> ExtendSegmentsToBoundariesAndIntersections(IEnumerable<Line> linesToExtend, Profile workingArea, double maxExtensionLength = 4.0)
        {
            return ExtendSegmentsToBoundariesAndIntersections(linesToExtend, workingArea, out var _, maxExtensionLength);
        }

        private static List<Line> ExtendSegmentsToBoundariesAndIntersections(IEnumerable<Line> linesToExtend, Profile workingArea, out IEnumerable<PointList> intersectionDebug, double maxExtensionLength = 4.0)
        {
            var extendedLines = new List<Line>();
            var intersectionPolylines = new List<PointList>();
            var obstructions = linesToExtend.Union(workingArea.Segments());
            // TODO - this could be made much more efficient
            int count = 0;
            foreach (var line in linesToExtend)
            {
                var testLine = new Line(line.PointAt(0.01), line.PointAt(0.99));
                var intersectionsForLine = new List<Vector3>();
                foreach (var obstruction in obstructions)
                {
                    try
                    {
                        bool pointAdded = false;
                        // special case for parallel + collinear lines
                        if (obstruction.Direction().IsParallelTo(testLine.Direction()) && // if the two lines are parallel
                            (new[] { obstruction.End, testLine.Start, testLine.End }).AreCollinear())// and collinear
                        {
                            if (!line.PointOnLine(obstruction.End, true))
                            {
                                intersectionsForLine.Add(obstruction.End);
                                pointAdded = true;
                            }

                            if (!line.PointOnLine(obstruction.Start, true))
                            {
                                intersectionsForLine.Add(obstruction.Start);
                                pointAdded = true;
                            }
                        }
                        if (!pointAdded)
                        {
                            var intersects = testLine.Intersects(obstruction, out Vector3 intersection, true);

                            // if the intersection lies on the obstruction, but is beyond the segment, we collect it
                            if (obstruction.PointOnLine(intersection) && !testLine.PointOnLine(intersection))
                            {
                                intersectionsForLine.Add(intersection);
                            }
                        }

                    }
                    catch (Exception e)
                    {
                        Console.WriteLine(e);
                    }
                }
                var dir = line.Direction();
                var intersectionsOrdered = intersectionsForLine.OrderBy(i => (testLine.Start - i).Dot(dir));
                var intersectionPolyline = new List<Vector3>(intersectionsOrdered.ToList());
                intersectionPolylines.Add(new PointList()
                {
                    index = count++,
                    Vertices = intersectionPolyline,
                    discriminator = "PointList"
                });
                var start = intersectionsOrdered.Where(i => (testLine.Start - i).Dot(dir) > 0).Cast<Vector3?>().FirstOrDefault() ?? line.Start;
                var end = intersectionsOrdered.Where(i => (testLine.Start - i).Dot(dir) < testLine.Length() * -1).Reverse().Cast<Vector3?>().FirstOrDefault() ?? line.End;
                var startClamped = maxExtensionLength > 0 && start.DistanceTo(line.Start) > maxExtensionLength ? line.Start : start;
                var endClamped = maxExtensionLength > 0 && end.DistanceTo(line.End) > maxExtensionLength ? line.End : end;
                extendedLines.Add(new Line(startClamped, endClamped));
            }
            intersectionDebug = intersectionPolylines;
            return extendedLines;
        }

        public static List<Line> DeduplicateSegments(IEnumerable<Line> segmentsExtended, Profile lockProfile, double tolerance = 0.2)
        {

            var allSegments = segmentsExtended;
            IEnumerable<Line> lockSegments = lockProfile.Perimeter.Segments();
            if (lockProfile.Voids != null)
            {
                lockSegments = lockSegments.Union(lockProfile.Voids.SelectMany(v => v.Segments()));
            }
            var refDir = new Vector3(1, 0, 0).Unitized(); // TODO: ensure no bias along refdir (e.g. counting parallel and antiparallel separately)
            var perpDir = refDir.Cross(Vector3.ZAxis);

            var resultSegments = new List<Line>();
            //group segments by angle

            var angleDict = GroupByAngle(allSegments, refDir);
            var lockAngleDict = GroupByAngle(lockSegments, refDir);

            //group by position along angle perpendicular
            var lineGroups = new List<List<List<(double pos, Line l)>>>();

            foreach (var angleGroup in angleDict)
            {
                var rotation = new Transform();
                rotation.Rotate(-angleGroup.Key);
                var perpDirRotated = rotation.OfVector(perpDir);

                IEnumerable<(double pos, Line l)> positions = angleGroup.Value.Select(line => (line.Start.Dot(perpDirRotated), line));

                if (lockAngleDict.ContainsKey(angleGroup.Key))
                {
                    IEnumerable<(double pos, Line l)> lockPositions = lockAngleDict[angleGroup.Key].Select(line => (line.Start.Dot(perpDirRotated), line));
                    List<(double pos, Line l)> cinchedPositions = new List<(double pos, Line l)>();
                    foreach ((double pos, Line l) in positions)
                    {
                        var closestLockedPosition = lockPositions.ClosestValue(pos);
                        if (Math.Abs(closestLockedPosition.pos - pos) < tolerance)
                        {
                            var projected = l.ProjectToLine(closestLockedPosition.l);
                            if (projected != null)
                            {
                                cinchedPositions.Add((closestLockedPosition.pos, projected));
                            }
                        }
                        else
                        {
                            cinchedPositions.Add((pos, l));
                        }
                    }
                    positions = cinchedPositions;
                }
                var positionsOrdered = positions.OrderBy(((double pos, Line l) p) => p.pos).ToList();

                var groupedLinesByPosition = GroupByPosition(positionsOrdered, tolerance);
                lineGroups.Add(groupedLinesByPosition);

                // project each group
                foreach (var group in groupedLinesByPosition)
                {

                    var linesOrderedByLength = group.Select(v => v.l).OrderByDescending(v => v.Length());
                    var dominantLineForGroup = linesOrderedByLength.First();
                    var domLineDir = dominantLineForGroup.Direction();

                    var orderEnds = new List<(double pos, bool isEnd)>();
                    var groupJson = JsonConvert.SerializeObject(group);
                    foreach (var line in group)
                    {
                        var start = (line.l.Start - dominantLineForGroup.Start).Dot(domLineDir);
                        var end = (line.l.End - dominantLineForGroup.Start).Dot(domLineDir);
                        if (start > end)
                        {
                            var oldStart = start;
                            start = end;
                            end = oldStart;
                        }

                        orderEnds.Add((start, false));
                        orderEnds.Add((end, true));
                    }

                    var count = 0;
                    double segmentStart = double.NaN;
                    // we should be guaranteed that the first point is a "start"
                    // since all segments are pointing in the direction of the dominant line.
                    var endsOrdered = orderEnds.OrderBy(e => e.pos);
                    foreach (var point in endsOrdered)
                    {
                        var prevCount = count;
                        count += point.isEnd ? -1 : 1;
                        if (count == 1 && prevCount == 0) // begin segment
                        {
                            segmentStart = point.pos;
                        }
                        if (count == 0) // end segment
                        {
                            var startPt = segmentStart * domLineDir + dominantLineForGroup.Start;
                            var endPt = point.pos * domLineDir + dominantLineForGroup.Start;
                            if (startPt.DistanceTo(endPt) > 0.01)
                            {
                                var newLine = new Line(startPt, endPt);
                                resultSegments.Add(newLine);
                            }
                        }
                    }
                }

            }
            return resultSegments;
        }

        private static Dictionary<int, List<Line>> GroupByAngle(IEnumerable<Line> allSegments, Vector3 refDir)
        {
            var angleDict = new Dictionary<int, List<Line>>();
            foreach (var segment in allSegments)
            {
                var realAngle = Math.Round(Angle_2D(segment.Direction(), refDir) * (360 / (2 * Math.PI)) + 360);
                var angle = (int)(realAngle) % 180;
                // var angle = Math.Round(segment.Direction().AngleTo(refDir)) % 180; // TODO - allow configuration of angle sensitivity;
                if (!angleDict.ContainsKey(angle))
                {
                    angleDict.Add(angle, new List<Line> { segment });
                }
                else
                {
                    angleDict[angle].Add(segment);
                }
            }

            return angleDict;
        }
        private static double Angle_2D(Vector3 A, Vector3 B)
        {
            // reject very small vectors
            if (A.Length() < Vector3.EPSILON || B.Length() < Vector3.EPSILON)
            {
                return double.NaN;
            }

            // project to XY Plane
            Vector3 aProjected = new Vector3(A.X, A.Y, 0).Unitized();
            Vector3 bProjected = new Vector3(B.X, B.Y, 0).Unitized();
            // Cos^-1(a dot b), a dot b clamped to [-1, 1]
            var num = Math.Acos(Math.Max(Math.Min(aProjected.Dot(bProjected), 1.0), -1.0));
            // Round close to 0 to 0
            if (Math.Abs(num) < Vector3.EPSILON)
            {
                return 0.0;
            }
            // Round close to pi to pi
            if (Math.Abs(num - Math.PI) < Vector3.EPSILON)
            {
                return Math.PI;
            }
            // check if should be reflex angle
            Vector3 aCrossB = aProjected.Cross(bProjected).Unitized();
            if (Vector3.ZAxis.Dot(aCrossB) > 0.999)
            {
                return num;
            }
            else
            {
                return Math.PI * 2 - num;
            }
        }

        private static List<List<(double pos, Line l)>> GroupByPosition(List<(double pos, Line l)> positionsOrdered, double tolerance = 0.5)
        {
            var buckets = new List<List<(double pos, Line l)>>();
            double lastAddedValue = double.MinValue;
            for (int i = 0; i < positionsOrdered.Count; i++)
            {
                if (positionsOrdered[i].pos - lastAddedValue < tolerance)
                {
                    buckets.Last().Add(positionsOrdered[i]);
                }
                else
                {
                    buckets.Add(new List<(double pos, Line l)> { positionsOrdered[i] });
                }
                lastAddedValue = positionsOrdered[i].pos;
            }

            for (int bucketIndex = 0; bucketIndex < buckets.Count; bucketIndex++)
            {
                var bucket = buckets[bucketIndex];
                if (bucket.Count == 1) continue;
                var bucketLength = bucket.Last().pos - bucket.First().pos;

                if (bucketLength < tolerance)
                {
                    var winningVal = bucket.OrderByDescending(b => b.l.Length()).First();

                    for (int i = 0; i < bucket.Count; i++)
                    {
                        bucket[i] = (winningVal.pos, bucket[i].l);
                    }
                    continue;
                }

                var valsToPlace = new List<(double pos, Line l)>(bucket);

                List<List<(double pos, Line l)>> replacementBuckets = new List<List<(double pos, Line l)>>();

                while (valsToPlace.Count > 0)
                {
                    // grab all the values within tolerance of start, and set them up as the current bucket.
                    var currBucket = valsToPlace.Where(v => v.pos - valsToPlace.First().pos < tolerance).ToList();
                    // remove them from the remaining vals to place
                    RemoveItems(valsToPlace, currBucket);
                    // create a new bucket for them
                    replacementBuckets.Add(currBucket);

                    // get the longest line in the bucket
                    var winningVal = currBucket.OrderByDescending(b => b.l.Length()).First();
                    // find any additional lines within tolerance of the longest line
                    var valsInSameBucket = valsToPlace.Where(v => v.pos - winningVal.pos < tolerance);
                    // remove these additional values from the vals to place
                    RemoveItems(valsToPlace, valsInSameBucket);
                    // add them to the current bucket
                    currBucket.AddRange(valsInSameBucket);
                    // project the current bucket so that all their positions are the same as the longest line
                    currBucket = currBucket.Select(v => (winningVal.pos, v.l)).ToList();
                }
                buckets.RemoveAt(bucketIndex);
                buckets.InsertRange(bucketIndex, replacementBuckets);
            }
            return buckets;
        }
        private static void RemoveItems(List<(double pos, Line l)> valsToPlace, IEnumerable<(double pos, Line l)> currBucket)
        {
            var posList = currBucket.Select(i => i.pos);
            for (int i = 0; i < valsToPlace.Count; i++)
            {
                (double pos, Line l) item = ((double pos, Line l))valsToPlace[i];
                if (posList.Contains(item.pos))
                {
                    valsToPlace.RemoveAt(i);
                }
            }
        }
        private static (double pos, Line l) ClosestValue(this IEnumerable<(double pos, Line l)> list, double number)
        {
            return list.Aggregate((x, y) => Math.Abs(x.pos - number) < Math.Abs(y.pos - number) ? x : y);
        }

        private static bool IsWithinProfile(Vector3 point, Profile workingArea)
        {
            if (workingArea.Voids != null)
            {
                foreach (var voidRegion in workingArea.Voids)
                {
                    if (voidRegion.Contains(point))
                    {
                        return false;
                    }
                }
            }
            return workingArea.Perimeter.Contains(point);
        }

        private static Vector3 ToVector3(this ng.Coordinate c)
        {
            return new Vector3(c.X, c.Y, 0);
        }

        private static ng.LineString ToLineString(this Line l)
        {
            return new ng.LineString(new[] { l.Start.ToCoordinate(), l.End.ToCoordinate() });
        }

        private static Polygon ToPolygon(this ng.Geometry g)
        {
            try
            {
                return new Polygon(g.Coordinates.Select(c => c.ToVector3()).Distinct().ToList());
            }
            catch
            {
                return null;
            }
        }

        private static ng.Coordinate ToCoordinate(this Vector3 v)
        {
            return new ng.Coordinate(v.X, v.Y);
        }
    }
}