using System.Collections.Generic;
using Elements.Geometry;
using Xunit;

namespace Elements.Tests
{
    public class TransformTests : ModelTest
    {
        [Fact, Trait("Category", "Examples")]
        public void Example()
        {
            this.Name = "Elements_Geometry_Transform";
            // <example>
            var m1 = new Mass(Polygon.Rectangle(1.0, 1.0), 1.0, new Material("yellow", Colors.Yellow));
            this.Model.AddElement(m1);

            Profile prof = Polygon.Rectangle(1.0, 1.0);

            var j = 1.0;
            var count = 10;
            for (var i = 0.0; i < 360.0; i += 360.0 / (double)count)
            {
                var m2 = new Mass(prof, 1.0, new Material($"color_{j}", new Color((float)j - 1.0f, 0.0f, 0.0f, 1.0f)), new Transform());

                // Scale the mass.
                m2.Transform.Scale(new Vector3(j, j, j));

                // Move the mass.
                m2.Transform.Move(new Vector3(3, 0, 0));

                // Rotate the mass.
                m2.Transform.Rotate(Vector3.ZAxis, i);
                this.Model.AddElement(m2);
                j += 1.0 / (double)count;
            }
            // </example>
        }

        [Fact]
        public void Transform_OfPoint()
        {
            var t = new Transform(Vector3.Origin, Vector3.XAxis, Vector3.YAxis.Negate());
            var v = new Vector3(0.5, 0.5, 0.0);
            var vt = t.OfPoint(v);
            Assert.Equal(0.5, vt.X);
            Assert.Equal(0.0, vt.Y);
            Assert.Equal(0.5, vt.Z);
        }

        [Fact]
        public void Involutary_Transform_Inverse()
        {
            // our library should handle involutary matrices well
            // https://en.wikipedia.org/wiki/Involutory_matrix 
            var origin = new Vector3(0, 0);
            var xAxis = new Vector3(-1, 0, 0);
            var zAxis = new Vector3(0, 1, 0);
            var transform = new Transform(origin, xAxis, zAxis);

            var inverted = new Transform(transform);
            inverted.Invert();

            Assert.Equal(transform, inverted);
        }


        [Fact]
        public void Transform_ScaleAboutPoint()
        {
            var transformOrigin = new Vector3(10, -5, 4);
            var pointToTransform = new Vector3(2, 9, 18);
            var scaleFactor = 2.5;
            var t = new Transform();
            t.Scale(scaleFactor, transformOrigin);
            var vt = t.OfPoint(pointToTransform);
            Assert.Equal(-10, vt.X);
            Assert.Equal(30, vt.Y);
            Assert.Equal(39, vt.Z);
        }

        [Fact]
        public void Transform_Translate()
        {
            var t = new Transform(new Vector3(5, 0, 0), Vector3.XAxis, Vector3.YAxis.Negate());
            var v = new Vector3(0.5, 0.5, 0.0);
            var vt = t.OfPoint(v);
            Assert.Equal(5.5, vt.X);
            Assert.Equal(0.0, vt.Y);
            Assert.Equal(0.5, vt.Z);
        }

        [Fact]
        public void TransformFromUp()
        {
            var t = new Transform(Vector3.Origin, Vector3.ZAxis);
            Assert.Equal(Vector3.XAxis, t.XAxis);
            Assert.Equal(Vector3.YAxis, t.YAxis);

            t = new Transform(Vector3.Origin, Vector3.XAxis);
            Assert.Equal(Vector3.YAxis, t.XAxis);
        }

        [Fact]
        public void OrientAlongCurve()
        {
            this.Name = "TransformsOrientedAlongCurve";
            var arc = new Arc(Vector3.Origin, 10.0, 45.0, 135.0);
            for (var i = 0.0; i <= 1.0; i += 0.1)
            {
                var t = Elements.Geometry.Transform.CreateOrientedAlongCurve(arc, i);
                var m = new Mass(Polygon.Rectangle(1.0, 1.0), 0.5, transform: t);
                this.Model.AddElement(m);
                this.Model.AddElements(t.ToModelCurves());
            }
        }

        [Fact]
        public void AllTransformsRunInTheSameDirection()
        {
            this.Name = "AllTransformsRunInTheSameDirection";

            var curves = new List<Curve>();

            var line = new Line(Vector3.Origin, new Vector3(5, 5, 5));
            curves.Add(line);

            var t = new Transform();
            t.Move(new Vector3(3, 0, 0));
            t.Rotate(Vector3.XAxis, 45);
            var l = Polygon.L(5, 3, 1);
            l = (Polygon)l.Transformed(t);
            curves.Add(l);

            var bez = new Bezier(new List<Vector3>(){
                new Vector3(10, 0, 0),
                new Vector3(10, 15, 5),
                new Vector3(10, 0, 10),
                new Vector3(10, 15, 15)
            }, FrameType.RoadLike);
            curves.Add(bez);

            var lp = Polygon.L(1, 1.5, 0.1);
            foreach (var curve in curves)
            {
                this.Model.AddElement(new Beam(curve, lp));
                this.Model.AddElement(new ModelCurve(curve));
                Transform last = null;
                for (var i = 0.0; i <= 1.0; i += 0.1)
                {
                    var tu = curve.TransformAt(i);
                    this.Model.AddElements(tu.ToModelCurves());
                    if (last == null)
                    {
                        last = tu;
                        continue;
                    }
                    // This test ensures that from one transform to
                    // the next, there is not a sudden flip in the
                    // Y vector.
                    Assert.True(tu.YAxis.Dot(last.YAxis) > 0);
                    last = tu;
                }
            }
        }
    }

}