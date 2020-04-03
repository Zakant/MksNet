using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using System.Xml;
using MksNet.Exceptions;
using MathNet.Numerics.LinearAlgebra;
using MksNet.Mbs.Elements;
using MksNet.Mbs.Parser.Internal;
using MksNet.Mbs.Joints;

namespace MksNet.Mbs.Parser
{
    internal static class MultibodySystemParser
    {
        internal static MultibodySystem Parse(string data)
        {
            var document = new XmlDocument();
            document.LoadXml(data);

            var system = new MultibodySystem();
            var root = document.DocumentElement;

            // Parse the gravity vector.
            var gravityNode = root.SelectSingleNode("Gravity");
            if (gravityNode == null)
                system.GravitationVector = CreateVector.Dense<double>(new double[] { 0, 0, -9.81 });
            else
                system.GravitationVector = DataParser.ParseVector(gravityNode.FirstChild).Resolve(null);

            // Parse global parameter.
            var globalParameter = ParseParameter(root.SelectSingleNode("Parameters"));

            // Parse elements
            var elementEntry = new List<(BodyEntry, Element)>();
            var elementNode = root.SelectSingleNode("Bodies");
            if (elementNode == null)
                throw new BadDefinitionException($"Error while parsing bodies. \"Bodies\" entry is missing.");
            if (elementNode.ChildNodes.Count == 0)
                throw new BadDefinitionException($"Error while parsing bodies. \"Bodies\" entry must have at least one body defined.");

            foreach (var child in elementNode.ChildNodes.Cast<XmlNode>())
            {
                var bodyEntry = ParseBody(child);
                var element = ElementFactory.Instance.Create(bodyEntry.Type, globalParameter + bodyEntry.LocalParameters);
                elementEntry.Add((bodyEntry, element));
            }

            // Connect the elements and create joints.
            var elements = new List<Element>();
            foreach (var (entry, element) in elementEntry)
            {
                // Create the joint.
                var joint = JointFactory.Instance.Create(entry.LinkType);

                // Find the parent element.
                if (entry.RemoteBody == "base")  // If the remote body is base, use the systems base frame.
                {
                    joint.BaseFrame = system.BaseFrame;
                    element.Parent = null;
                }
                else  // Find the correct body.
                {
                    var parentEntry = elementEntry.Where(x => x.Item1.Name == entry.RemoteBody).FirstOrDefault();
                    element.Parent = parentEntry.Item2;
                    if (element.Parent == null)
                        throw new BadDefinitionException($"Error while parsing link. Parent \"{entry.RemoteBody}\" was not found.");
                    if (element.Parent == element)
                        throw new BadDefinitionException($"Error while parsing link. Link can not have same local and remote body.");
                    joint.BaseFrame = element.Parent.Frames.Where(x => x.Name == entry.RemoteFrame).FirstOrDefault();
                    if (joint.BaseFrame == null)
                        throw new BadDefinitionException($"Error while creating joint. Frame\"{entry.RemoteFrame}\" was not found on body of type \"{parentEntry.Item1.Type}\".");
                }
                // Assign the joint properties.
                joint.FollowerFrame = element.Frames.Where(x => x.Name == entry.LocalFrame).FirstOrDefault();
                if (joint.FollowerFrame == null)
                    throw new BadDefinitionException($"Error while creating joint. Frame\"{entry.LocalFrame}\" was not found on body of type \"{entry.Type}\".");

                element.BaseJoint = joint;
                elements.Add(element);
            }

            // Sort elements
            var resultList = new List<Element>(elements.Count);
            int index = 0;
            while (elements.Count > 0)
            {
                if (resultList.Contains(elements[index].Parent) || elements[index].Parent == null)
                {
                    resultList.Add(elements[index]);
                    elements.Remove(elements[index]);
                    index = 0;
                }
                if (index >= elements.Count)
                    index = 0;
                else
                    index++;
            }

            system.Elements = resultList;
            system.InitilizeSystem();
            return system;
        }

        internal static Parameter ParseParameter(XmlNode node)
        {
            var param = new Parameter();
            if (node != null)
                foreach (var child in node.ChildNodes.Cast<XmlNode>())
                    switch (child.Name)
                    {
                        case "ScalarParameter":
                            param.Add(child.Attributes["name"].Value, DataParser.ParseScalar(child.FirstChild).Resolve(null));
                            break;
                        case "VectorParameter":
                            param.Add(child.Attributes["name"].Value, DataParser.ParseVector(child.FirstChild).Resolve(null));
                            break;
                        case "MatrixParameter":
                            param.Add(child.Attributes["name"].Value, DataParser.ParseMatrix(child.FirstChild).Resolve(null));
                            break;
                        default:
                            throw new BadDefinitionException($"Error while parsing parameters. Encountered unknown parameter type \"{child.Name}\".");
                    }
            return param;
        }

        private static BodyEntry ParseBody(XmlNode node)
        {
            if (node == null)
                throw new ArgumentNullException(nameof(node));
            var entry = new BodyEntry();
            entry.Name = node.Attributes["name"].Value.ToLower();
            entry.Type = node.Attributes["type"].Value.ToLower();

            var parameterNode = node.SelectSingleNode("Parameters");
            if (parameterNode == null)
                entry.LocalParameters = new Parameter();
            else
                entry.LocalParameters = ParseParameter(parameterNode);

            var linkNode = node.SelectSingleNode("Link");
            if (linkNode == null)
                throw new BadDefinitionException($"Error while parsing Body. Body must have exactly one \"Link\" entry.");
            entry.LinkType = linkNode.Attributes["type"].Value.ToLower();
            entry.LocalFrame = linkNode.Attributes["localframe"].Value.ToLower();

            var remoteText = linkNode.Attributes["remote"].Value.ToLower().Split('/');
            entry.RemoteBody = remoteText[0];
            entry.RemoteFrame = remoteText.Length > 1 ? remoteText[1] : "origin";

            return entry;
        }
    }
}
