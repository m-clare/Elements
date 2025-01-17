//----------------------
// <auto-generated>
//     Generated using the NJsonSchema v10.1.21.0 (Newtonsoft.Json v11.0.0.0) (http://NJsonSchema.org)
// </auto-generated>
//----------------------
using Elements;
using Elements.GeoJSON;
using Elements.Geometry;
using Elements.Geometry.Solids;
using Elements.Spatial;
using Elements.Validators;
using Elements.Serialization.JSON;
using System;
using System.Collections.Generic;
using System.Linq;
using Line = Elements.Geometry.Line;
using Polygon = Elements.Geometry.Polygon;

namespace Elements
{
    #pragma warning disable // Disable all warnings

    /// <summary>An element representing user content.</summary>
    [Newtonsoft.Json.JsonConverter(typeof(Elements.Serialization.JSON.JsonInheritanceConverter), "discriminator")]
    [System.CodeDom.Compiler.GeneratedCode("NJsonSchema", "10.1.21.0 (Newtonsoft.Json v11.0.0.0)")]
    public partial class ContentElement : GeometricElement
    {
        [Newtonsoft.Json.JsonConstructor]
        public ContentElement(string @gltfLocation, BBox3 @boundingBox, double @gltfScaleToMeters, Vector3 @sourceDirection, Transform @transform, Material @material, Representation @representation, bool @isElementDefinition, System.Guid @id, string @name)
            : base(transform, material, representation, isElementDefinition, id, name)
        {
            var validator = Validator.Instance.GetFirstValidatorForType<ContentElement>();
            if(validator != null)
            {
                validator.PreConstruct(new object[]{ @gltfLocation, @boundingBox, @gltfScaleToMeters, @sourceDirection, @transform, @material, @representation, @isElementDefinition, @id, @name});
            }
        
            this.GltfLocation = @gltfLocation;
            this.BoundingBox = @boundingBox;
            this.GltfScaleToMeters = @gltfScaleToMeters;
            this.SourceDirection = @sourceDirection;
            
            if(validator != null)
            {
                validator.PostConstruct(this);
            }
        }
    
        /// <summary>The URI of the gltf for this element.</summary>
        [Newtonsoft.Json.JsonProperty("gltfLocation", Required = Newtonsoft.Json.Required.DisallowNull, NullValueHandling = Newtonsoft.Json.NullValueHandling.Ignore)]
        public string GltfLocation { get; set; }
    
        /// <summary>The bounding box of the content.</summary>
        [Newtonsoft.Json.JsonProperty("Bounding Box", Required = Newtonsoft.Json.Required.DisallowNull, NullValueHandling = Newtonsoft.Json.NullValueHandling.Ignore)]
        public BBox3 BoundingBox { get; set; }
    
        /// <summary>The scale needed to convert the gltf to meters.</summary>
        [Newtonsoft.Json.JsonProperty("Gltf Scale to Meters", Required = Newtonsoft.Json.Required.DisallowNull, NullValueHandling = Newtonsoft.Json.NullValueHandling.Ignore)]
        public double GltfScaleToMeters { get; set; }
    
        /// <summary>A vector indicating the direction the source object was originally facing.</summary>
        [Newtonsoft.Json.JsonProperty("SourceDirection", Required = Newtonsoft.Json.Required.Default, NullValueHandling = Newtonsoft.Json.NullValueHandling.Ignore)]
        public Vector3 SourceDirection { get; set; }
    
        private System.Collections.Generic.IDictionary<string, object> _additionalProperties = new System.Collections.Generic.Dictionary<string, object>();
    
        [Newtonsoft.Json.JsonExtensionData]
        public System.Collections.Generic.IDictionary<string, object> AdditionalProperties
        {
            get { return _additionalProperties; }
            set { _additionalProperties = value; }
        }
    
    
    }
}