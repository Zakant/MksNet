using MksNet.Elements.Xml.Operations.Matrix;
using MksNet.Elements.Xml.Operations.Scalar;
using MksNet.Elements.Xml.Operations.Vector;
using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using System.Xml;
using MksNet.Exceptions;

namespace MksNet.Elements.Xml
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
            entry.MassOperation = ParseScalar(propertyNode.SelectSingleNode("Mass").FirstChild);

            var inertiaChildrenCount = propertyNode.SelectSingleNode("Inertia")?.ChildNodes.Count;
            if (inertiaChildrenCount == null || inertiaChildrenCount != 1)
                throw new BadDefinitionException($"Error during parsing inertia property. Either the property is missing, has zero or more the one entry.");
            entry.InertiaOperation = ParseMatrix(propertyNode.SelectSingleNode("Inertia").FirstChild);

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
                        frameEntry.TranslationOperation = ParseVector(translationNode.FirstChild);
                    else
                        frameEntry.TranslationOperation = new ZeroVectorOperation();

                    var rotationNode = frame.SelectSingleNode("Rotation");
                    if (rotationNode != null)
                        frameEntry.RotationOperation = ParseMatrix(rotationNode.FirstChild);
                    else
                        frameEntry.RotationOperation = new IdentityMatrixOperation();


                    entry.Frames.Add(frameEntry);
                }

            return entry;
        }

        internal static IScalarOperation ParseScalar(XmlNode node)
        {
            return node?.Name switch
            {
                "Parameter" => new ScalarParameterOperation(node.Attributes["name"].Value),
                "Number" => new NumberOperation(double.Parse(node.InnerText)),
                "Zero" => new NumberOperation(0),
                "Rad2Deg" => new Rad2DegOperation(ParseScalar(node.FirstChild)),
                "Deg2Rad" => new Deg2RadOperation(ParseScalar(node.FirstChild)),
                "Sin" => new SinOperation(ParseScalar(node.FirstChild)),
                "Cos" => new CosOperation(ParseScalar(node.FirstChild)),
                "Add" => new AddOperation(ParseAllChildren(node)),
                "Subtract" => new SubtractOperation(ParseAllChildren(node)),
                "Multiply" => new MultiplyOperation(ParseAllChildren(node)),
                "Divide" => new DivideOperation(ParseAllChildren(node)),
                null => throw new BadDefinitionException($"Error while parsing scalar. No child was found."),
                _ => throw new BadDefinitionException("Error while parsing scalar. Unknown type encounterd.")
            };

            static List<IScalarOperation> ParseAllChildren(XmlNode node)
            {
                var children = node.ChildNodes;
                if (children.Count < 2)
                    throw new BadDefinitionException("Error while parsing scalar. Operations must have atleast two children.");
                List<IScalarOperation> operations = new List<IScalarOperation>();
                foreach (var child in children.Cast<XmlNode>())
                    operations.Add(ParseScalar(child));
                return operations;
            }
        }

        internal static IVectorOperation ParseVector(XmlNode node)
        {
            return node?.Name switch
            {
                "Parameter" => new VectorParameterOperation(node.Attributes["name"].Value),
                "Vector" => ParseVectorEntry(node),
                null => throw new BadDefinitionException("Error while parsing vector. No child was found."),
                _ => throw new BadDefinitionException("Error while parsing vector. Unknown type encounterd.")
            };

            static IVectorOperation ParseVectorEntry(XmlNode node)
            {
                if (node.ChildNodes.Count != 3)
                    throw new BadDefinitionException("Error while parsing vector. Vectors must have exact three elements.");

                return new VectorOperation((new string[] { "X", "Y", "Z" }).Select(entry =>
                {
                    var entryNode = node.SelectSingleNode(entry);
                    if (entryNode == null)
                        throw new BadDefinitionException($"Error while parsing vector. No element named \"{entry}\" was found.");
                    if (entryNode.ChildNodes.Count != 1)
                        throw new BadDefinitionException($"Error during parsing vector element \"{entry}\". The element must have exactly one children.");
                    return ParseScalar(entryNode.FirstChild);
                }));
            }
        }

        internal static IMatrixOperation ParseMatrix(XmlNode node)
        {
            return node?.Name switch
            {
                "Parameter" => new MatrixParameterOperation(node.Attributes["name"].Value),
                "Matrix" => ParseMatrixEntry(node),
                null => throw new BadDefinitionException("Error while parsing matrix. No child was found."),
                _ => throw new BadDefinitionException("Error while parsing matrix. Unknown type encounterd!")
            };

            static IMatrixOperation ParseMatrixEntry(XmlNode node)
            {
                if (node.ChildNodes.Count == 0)
                    throw new BadDefinitionException("Error while parsing matrix. Matrix must have atleast one row.");

                var result = node.ChildNodes.Cast<XmlNode>().Select(rowNode =>
                {
                    if (rowNode.Name != "Row")
                        throw new BadDefinitionException($"Error while parsing matrix. Matrix can only have children named \"Row\". Encountered {rowNode.Name}.");
                    if (rowNode.ChildNodes.Count == 0)
                        throw new BadDefinitionException($"Error while parsing matrix. Rows must have atleast one child.");

                    return rowNode.ChildNodes.Cast<XmlNode>().Select(numberNode => ParseScalar(numberNode));
                });
                var columnValues = result.Select(x => x.Count()).Distinct().ToList();
                if (columnValues.Count > 1)
                    throw new BadDefinitionException($"Error while parsing matrix. Rows must have same length. Found length was: {String.Join(", ", columnValues)}.");

                return new MatrixOperation(result);
            }
        }
    }
}
