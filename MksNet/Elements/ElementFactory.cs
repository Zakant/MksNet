using MksNet.Exceptions;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace MksNet.Elements
{
    /// <summary>
    /// Factory class to create <see cref="Element"/> objects.
    /// </summary>
    public sealed class ElementFactory
    {

        /// <summary>
        /// File extension for element definition files.
        /// </summary>
        private const string FILE_EXTENSION = "*.edf";

        /// <summary>
        /// Get the singelton instace of the element factory.
        /// </summary>
        public static ElementFactory Instance { get; } = new ElementFactory();


        private Dictionary<string, ElementFactoryEntry> factoryEntries = new Dictionary<string, ElementFactoryEntry>();

        private ElementFactory() { }

        /// <summary>
        /// Create a element from the given type name.
        /// </summary>
        /// <param name="name">The name of the element type.</param>
        /// <returns>The element instance</returns>
        public Element Create(string name)
        {
            if (!factoryEntries.ContainsKey(name.ToLower()))
                throw new ElementNotFoundException($"No element type with name \"{name}\" is registered.");

            var entry = factoryEntries[name.ToLower()];

            var element = new Element();

            throw new NotImplementedException();

            return element;
        }

        /// <summary>
        /// Load a element definition from the given text reader and adds it to the factory.
        /// </summary>
        /// <param name="reader">Text reader used to provide element definition.</param>
        public void LoadDefinition(TextReader reader)
        {
            throw new NotImplementedException();
            // Perform the loading step.
            // var serializer = new XmlSerializer(typeof(Xml.XXX));
            // var definition = (XXX)serializer.Deserialize(reader);


            // if (factoryEntries.ContainsKey(definition.Name.ToLower()))
            //    throw new InvalidOperationException($"Error while adding {definition.Name}. A definition with the same was already added!");
            // factoryEntries.Add(definition.Name.ToLower(), entry);
        }

        /// <summary>
        /// Load a element definition from a file and adds it to the factory.
        /// </summary>
        /// <param name="filePath">The file path.</param>
        public void LoadDefinition(string filePath) => AddDefinition(File.ReadAllText(filePath));

        /// <summary>
        /// Load all element definitions from the given folder and adds them to the factory.
        /// </summary>
        /// <param name="folderPath">Path to the defintion folder.</param>
        public void LoadFolderDefinition(string folderPath)
        {
            foreach (var file in Directory.GetFiles(folderPath, FILE_EXTENSION))
                LoadDefinition(file);
        }

        /// <summary>
        /// Add a element definition the the factory.
        /// </summary>
        /// <param name="definition">The definition string.</param>
        public void AddDefinition(string definition)
        {
            using (var reader = new StringReader(definition))
                LoadDefinition(reader);
        }
    }
}
