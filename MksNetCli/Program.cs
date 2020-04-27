using MksNet.Mbs;
using MksNet.Mbs.Elements;
using MksNet.Mbs.Joints;
using System;
using MathNet.Numerics.LinearAlgebra;

namespace MksNetCli
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello World!");

            ElementFactory.Instance.LoadFolderDefinition(@".\Definitions\Elements");
            JointFactory.Instance.LoadFolderDefinition(@".\Definitions\Joints");
            MultibodySystem MBS = MultibodySystem.LoadFromFile(@".\Definitions\Systems\SimplePendulum.xml");
            double[] vec = new double[MBS.Elements.Count * 6 * 2];
            StateVector SV = new StateVector(vec, MBS.GenerateMappings());
            MBS.GetGlobalKeepMatrix();
            MBS.SetupElements();
            MBS.UpdateElements(SV);
            Matrix<double> GlobalMassMatrix = CreateMatrix.Dense<double>(MBS.Elements.Count * 6, MBS.Elements.Count * 6);
            GlobalMassMatrix = MBS.GetGlobalMassMatrix(GlobalMassMatrix);
            Matrix<double> GlobalJacobian = CreateMatrix.Dense<double>(MBS.Elements.Count * 6, MBS.Elements.Count * 6);
            Vector<double> GlobalAngularVelocity = CreateVector.Dense<double>(6);

            GlobalJacobian = MBS.GetGlobalJacobian(GlobalJacobian);
            
        }
    }
}
