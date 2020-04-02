using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using System.Xml;
using MksNet.Exceptions;
using MksNet.Mbs.Parser.Operations.Vector;
using MksNet.Mbs.Parser.Operations.Matrix;

using MksNet.Mbs.Parser;

namespace MksNet.Mbs.Elements.Xml
{
    internal static class ElementFileParser
    {

        internal static ElementFactoryEntry Parse(XmlDocument document)
        {
            var entry = new ElementFactoryEntry();
            var root = document.DocumentElement;

            // Load the header data
            entry.Name = root.SelectSingleNode("Name").InnerText;
            entry.Author = root.SelectSingleNode("Author").InnerText;
            entry.Description = root.SelectSingleNode("Description")?.InnerText ?? "";
            entry.Url = root.SelectSingleNode("URL")?.InnerText ?? "";


            // Load the properties.
            var propertyNode = root.SelectSingleNode("Properties");

            var massChildrenCount = propertyNode.SelectSingleNode("Mass")?.ChildNodes.Count;
            if (massChildrenCount == null || massChildrenCount != 1)
                throw new BadDefinitionException($"Error during parsing mass property. Either the property is missing, has zero or more the one entry.");
            entry.MassOperation = DataParser.ParseScalar(propertyNode.SelectSingleNode("Mass").FirstChild);

            var inertiaChildrenCount = propertyNode.SelectSingleNode("Inertia")?.ChildNodes.Count;
            if (inertiaChildrenCount == null || inertiaChildrenCount != 1)
                throw new BadDefinitionException($"Error during parsing inertia property. Either the property is missing, has zero or more the one entry.");
            entry.InertiaOperation = DataParser.ParseMatrix(propertyNode.SelectSingleNode("Inertia").FirstChild);

            // Parse frames
            entry.Frames = new List<ElementFactoryEntryFrame>();
            var framesNode = root.SelectSingleNode("Frames");
            if (framesNode != null)
                foreach (var frame in framesNode.ChildNodes.Cast<XmlNode>())
                {
                    var frameEntry = new ElementFactoryEntryFrame();

                    frameEntry.Name = frame.Attributes["name"].Value;
                    frameEntry.Reference = frame.Attributes["reference"]?.Value ?? "origin";

                    var translationNode = frame.SelectSingleNode("Translation");
                    if (translationNode != null)
                        frameEntry.TranslationOperation = DataParser.ParseVector(translationNode.FirstChild);
                    else
                        frameEntry.TranslationOperation = new ZeroVectorOperation();

                    var rotationNode = frame.SelectSingleNode("Rotation");
                    if (rotationNode != null)
                        frameEntry.RotationOperation = DataParser.ParseMatrix(rotationNode.FirstChild);
                    else
                        frameEntry.RotationOperation = new IdentityMatrixOperation();


                    entry.Frames.Add(frameEntry);
                }

            return entry;
        }
    }
}
