using MksNet.Exceptions;
using MksNet.Mbs.Parser;
using MksNet.Mbs.Parser.Internal;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Linq;
using System.Xml;

namespace MksNet.Mbs.Elements
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
        /// Create an element from the given type name with empty parameters.
        /// </summary>
        /// <param name="name">The name of the element type.</param>
        /// <returns>The element instance.</returns>
        public Element Create(string name) => Create(name, Parameter.Empty);

        /// <summary>
        /// Create an element from the given type name and the given parameters.
        /// </summary>
        /// <param name="name">The name of the element type.</param>
        /// <param name="parameter">Parameters used for creation.</param>
        /// <returns>The element instance.</returns>
        public Element Create(string name, Parameter parameter)
        {
            if (!factoryEntries.ContainsKey(name.ToLower()))
                throw new ElementNotFoundException($"No element type with name \"{name}\" is registered.");

            var entry = factoryEntries[name.ToLower()];

            var element = new Element();

            element.Mass = entry.MassOperation.Resolve(parameter);
            element.Inertia = entry.InertiaOperation.Resolve(parameter);

            element.Origin = new OriginFrame();

            var frameDict = new Dictionary<string, (string, Frame)>();
            frameDict.Add("origin", ("", element.Origin));

            element.Cog = element.Origin; // For now, assume cog is origin.

            foreach (var frameEntry in entry.Frames)
            {
                var frame = new Frame(null);
                frame.Name = frameEntry.Name;
                frame.Offset = frameEntry.TranslationOperation.Resolve(parameter);
                frame.Rotation = frameEntry.RotationOperation.Resolve(parameter);
                frameDict.Add(frameEntry.Name, (frameEntry.Reference, frame));
            }

            foreach (var frameEntry in frameDict)
            {
                if (frameEntry.Value.Item1 == "")
                    continue;
                frameEntry.Value.Item2.Reference = frameDict[frameEntry.Value.Item1].Item2;
            }

            element.Frames = frameDict.Select(x => x.Value.Item2).ToList();

            return element;
        }

        /// <summary>
        /// Load a element definition from the given text reader and adds it to the factory.
        /// </summary>
        /// <param name="reader">Text reader used to provide element definition.</param>
        public void LoadDefinition(TextReader reader)
        {
            var doc = new XmlDocument();
            doc.LoadXml(reader.ReadToEnd());
            var entry = ElementFileParser.Parse(doc);

            if (factoryEntries.ContainsKey(entry.Name.ToLower()))
                throw new InvalidOperationException($"Error while adding {entry.Name}. A definition with the same was already added!");

            factoryEntries.Add(entry.Name.ToLower(), entry);
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
