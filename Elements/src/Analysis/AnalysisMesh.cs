using System;
using System.Collections.Generic;
using Elements.Geometry;
using Elements.Geometry.Interfaces;
using Newtonsoft.Json;

namespace Elements.Analysis
{
    /// <summary>
    /// A visualization of computed values at locations in space.
    /// </summary>
    /// <example>
    /// [!code-csharp[Main](../../Elements/test/AnalysisMeshTests.cs?name=example)]
    /// </example>
    public class AnalysisMesh : GeometricElement, ITessellate
    {
        private List<(BBox3 cell, double value)> _results = new List<(BBox3 cell, double value)>();
        private Func<Vector3, double> _analyze;
        private double _min = double.MaxValue;
        private double _max = double.MinValue;

        /// <summary>
        /// The total number of analysis locations.
        /// </summary>
        [JsonIgnore]
        public double TotalAnalysisLocations
        {
            get { return _results.Count; }
        }

        /// <summary>
        /// The length of the cells in the u direction.
        /// </summary>
        public double ULength { get; set; }

        /// <summary>
        /// The length of the cells in the v direction.
        /// </summary>
        public double VLength { get; set; }

        /// <summary>
        /// The perimeter of the analysis mesh.
        /// </summary>
        public Polygon Perimeter { get; set; }

        /// <summary>
        /// The color scale used to represent this analysis mesh.
        /// </summary>
        public ColorScale ColorScale { get; set; }

        /// <summary>
        /// Construct an analysis mesh.
        /// </summary>
        /// <param name="perimeter">The perimeter of the mesh.</param>
        /// <param name="uLength">The number of divisions in the u direction.</param>
        /// <param name="vLength">The number of divisions in the v direction.</param>
        /// <param name="colorScale">The color scale to be used in the visualization.</param>
        /// <param name="analyze">A function which takes a location and computes a value.</param>
        /// <param name="id">The id of the analysis mesh.</param>
        /// <param name="name">The name of the analysis mesh.</param>
        public AnalysisMesh(Polygon perimeter,
                            double uLength,
                            double vLength,
                            ColorScale colorScale,
                            Func<Vector3, double> analyze,
                            Guid id = default(Guid),
                            string name = null) : base(new Transform(),
                                                       BuiltInMaterials.Default,
                                                       null,
                                                       false,
                                                       id == default(Guid) ? Guid.NewGuid() : id,
                                                       name)
        {
            this.Perimeter = perimeter;
            this.ULength = uLength;
            this.VLength = vLength;
            this.ColorScale = colorScale;
            this._analyze = analyze;
            this.Material = new Material($"Analysis_{Guid.NewGuid().ToString()}", Colors.White, 0, 0, null, true, true, Guid.NewGuid());
        }

        /// <summary>
        /// Tessellate the analysis mesh.
        /// </summary>
        /// <param name="mesh"></param>
        /// <param name="transform"></param>
        /// <param name="color"></param>
        public void Tessellate(ref Mesh mesh, Transform transform = null, Color color = default(Color))
        {
            var span = this._max - this._min;

            foreach (var result in this._results)
            {
                var center = result.cell.Center();
                var vertexColor = this.Perimeter.Contains(center) ? this.ColorScale.GetColor((result.value - this._min) / span) : Colors.White;
                var min = result.cell.Min;
                var max = result.cell.Max;
                var v1 = mesh.AddVertex(min, color: vertexColor);
                var v2 = mesh.AddVertex(new Vector3(max.X, min.Y), color: vertexColor);
                var v3 = mesh.AddVertex(max, color: vertexColor);
                var v4 = mesh.AddVertex(new Vector3(min.X, max.Y), color: vertexColor);
                mesh.AddTriangle(v1, v2, v3);
                mesh.AddTriangle(v3, v4, v1);
            }

            mesh.ComputeNormals();
        }

        /// <summary>
        /// Compute a value for each grid cell.
        /// </summary>
        public void Analyze()
        {
            var bounds = new BBox3(new[] { this.Perimeter });
            var w = bounds.Max.X - bounds.Min.X;
            var h = bounds.Max.Y - bounds.Min.Y;
            var x = bounds.Min.X;
            var y = bounds.Min.Y;

            while (x <= bounds.Min.X + w)
            {
                while (y <= bounds.Min.Y + h)
                {
                    var cell = new BBox3(new Vector3(x, y), new Vector3(x + this.ULength, y + this.VLength));
                    var result = this._analyze(cell.Center());
                    this._min = Math.Min(this._min, result);
                    this._max = Math.Max(this._max, result);
                    this._results.Add((cell, result));
                    y += this.VLength;
                }
                y = bounds.Min.Y;
                x += this.ULength;
            }
        }
    }
}