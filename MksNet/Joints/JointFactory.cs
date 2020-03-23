using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Xml.Serialization;

namespace MksNet.Joints
{
    /// <summary>
    /// Factory class to create <see cref="Joint"/> objects.
    /// </summary>
    public sealed class JointFactory
    {
        /// <summary>
        /// Get the singelton instace of the joint factory.
        /// </summary>
        public static JointFactory Instance { get; } = new JointFactory();


        private Dictionary<string, JointFactoryEntry> factoryEntries = new Dictionary<string, JointFactoryEntry>();

        private JointFactory() { }

        /// <summary>
        /// Create a joint with the given type name.
        /// </summary>
        /// <param name="name">The name of the joint type.</param>
        /// <returns>The joint instance</returns>
        public Joint Create(string name)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Load a joint definition from the given text reader and adds it to the factory.
        /// </summary>
        /// <param name="reader">Text reader used to provide joint definition.</param>
        public void LoadDefintion(TextReader reader)
        {
            // Perform the loading step.
            var serializer = new XmlSerializer(typeof(Xml.JointDefinition));
            var definition = (Xml.JointDefinition)serializer.Deserialize(reader);

            // Add the result to the factory.
            var entry = new JointFactoryEntry();

            entry.Name = definition.Name.ToUpperInvariant();
            entry.Author = definition.Author;
            entry.Description = definition.Description;
            entry.Url = definition.URL;

            List<DOF> defaultList;
            if (definition.DegreesOfFreedom.@default == Xml.JointDefinitionDegreesOfFreedomDefault.free)
                defaultList = entry.FreeDofs;
            else
                defaultList = entry.LockedDofs;

            // Add all dofs to default.
            foreach (var x in (DOF[])Enum.GetValues(typeof(DOF)))
                defaultList.Add(x);

            // Move the specified dofs
            foreach(var x in definition.DegreesOfFreedom.Items)
            {
                if (x is Xml.JointDefinitionDegreesOfFreedomFree)
                {
                    var type = (DOF)Enum.Parse(typeof(DOF), (x as Xml.JointDefinitionDegreesOfFreedomFree).type.ToString());
                    defaultList.Remove(type);
                    entry.FreeDofs.Add(type);
                }
                else
                {
                    var type = (DOF)Enum.Parse(typeof(DOF), (x as Xml.JointDefinitionDegreesOfFreedomLocked).type.ToString());
                    defaultList.Remove(type);
                    entry.LockedDofs.Add(type);
                }
            }

            if (factoryEntries.ContainsKey(definition.Name.ToLower()))
                throw new InvalidOperationException($"Error while adding {definition.Name}. A definition with the same was already added!");
            factoryEntries.Add(definition.Name.ToLower(), entry);
        }

        /// <summary>
        /// Load a joint definition from a file and adds it to the factory.
        /// </summary>
        /// <param name="filePath">The file path.</param>
        public void LoadDefinition(string filePath) => AddDefinition(File.ReadAllText(filePath));

        /// <summary>
        /// Load all joint definitions from the given folder and adds them to the factory.
        /// </summary>
        /// <param name="folderPath">Path to the defintion folder.</param>
        public void LoadFolderDefinition(string folderPath)
        {
            foreach (var file in Directory.GetFiles(folderPath, "*.jdf"))
                LoadDefinition(file);
        }

        /// <summary>
        /// Add a joint definition the the factory.
        /// </summary>
        /// <param name="definition">The definition string.</param>
        public void AddDefinition(string definition)
        {

        }
    }
}
