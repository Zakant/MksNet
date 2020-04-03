using MksNet.Exceptions;
using MksNet.Mbs.Parser.Operations.Matrix;
using MksNet.Mbs.Parser.Operations.Scalar;
using MksNet.Mbs.Parser.Operations.Vector;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Xml;

namespace MksNet.Mbs.Parser
{
    internal static class DataParser
    {
        internal static IScalarOperation ParseScalar(XmlNode node)
        {
            return node?.Name switch
            {
                "Parameter" => new ScalarParameterOperation(node.Attributes["name"].Value),
                "Number" => new NumberOperation(double.Parse(node.InnerText, CultureInfo.InvariantCulture)),
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

                return new VectorOperation(node.ChildNodes.Cast<XmlNode>().Select(x => ParseScalar(x)));
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
