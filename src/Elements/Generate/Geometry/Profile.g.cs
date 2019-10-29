//----------------------
// <auto-generated>
//     Generated using the NJsonSchema v10.0.27.0 (Newtonsoft.Json v12.0.0.0) (http://NJsonSchema.org)
// </auto-generated>
//----------------------
using Elements;
using Elements.GeoJSON;
using Elements.Geometry;
using Elements.Geometry.Solids;
using Elements.Properties;
using System;
using System.Collections.Generic;
using System.Linq;
using Line = Elements.Geometry.Line;
using Polygon = Elements.Geometry.Polygon;

namespace Elements.Geometry
{
    #pragma warning disable // Disable all warnings

    /// <summary>A profile comprised of an external boundary and one or several holes.</summary>
    [System.CodeDom.Compiler.GeneratedCode("NJsonSchema", "10.0.27.0 (Newtonsoft.Json v12.0.0.0)")]
    public partial class Profile : Identifiable
    {
        [Newtonsoft.Json.JsonConstructor]
        public Profile(Polygon @perimeter, IList<Polygon> @voids, System.Guid @id, string @name)
            : base(id, name)
        {
            Profile.ValidateConstructorParameters(@perimeter, @voids, @id, @name);
        
            this.Perimeter = @perimeter;
            this.Voids = @voids;
        }
    
        /// <summary>The perimeter of the profile.</summary>
        [Newtonsoft.Json.JsonProperty("Perimeter", Required = Newtonsoft.Json.Required.AllowNull)]
        public Polygon Perimeter { get; internal set; }
    
        /// <summary>A collection of Polygons representing voids in the profile.</summary>
        [Newtonsoft.Json.JsonProperty("Voids", Required = Newtonsoft.Json.Required.AllowNull)]
        public IList<Polygon> Voids { get; internal set; }
    
        private System.Collections.Generic.IDictionary<string, object> _additionalProperties = new System.Collections.Generic.Dictionary<string, object>();
    
        [Newtonsoft.Json.JsonExtensionData]
        public System.Collections.Generic.IDictionary<string, object> AdditionalProperties
        {
            get { return _additionalProperties; }
            set { _additionalProperties = value; }
        }
    
    
    }
}