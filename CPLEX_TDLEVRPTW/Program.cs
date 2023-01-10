// See https://aka.ms/new-console-template for more information
using CPLEX_TDTSPTW;
using System.Globalization;

//Culture is included to get the directory where Config and data is placed
CultureInfo ci = new CultureInfo("hr-HR");
Thread.CurrentThread.CurrentCulture = ci;
Thread.CurrentThread.CurrentUICulture = ci;
string mainDir = Directory.GetCurrentDirectory();
int ind = mainDir.IndexOf("bin");
if (ind != -1)
{
    mainDir = mainDir.Remove(ind - 1);
}

//Path to config and data directory that contains all the input data
string pathCFG = mainDir + "\\Config and data\\";
Params p = new Params(mainDir, pathCFG + "cfg.txt");

//Stream for output file
StreamWriter ofsFinal = new StreamWriter(p.outputFileName + "_" + p.minimizationType.ToString() + "_" + p.travelTimeCompType + ".txt");

//Stream for input file containing all small instances and set  values (schneider 2014)
StreamReader ifs = new StreamReader(pathCFG + p.instancesFile);
//Delimited
string d = ";";
//Header in output line
string headerLine = "INSATNCE_NAME" + d + "NUM_CUST" + d + "SCHNEIDER_VEH" + d + "SCHNEIDER_DIST" + d + "BETA" + d +
    "KNOWN_VEH_NUM_CPLEX" + d + "MINIMIZATION_TYPE" + d + "CPLEX_MIN_VEH_NUM" + d + "CPLEX_VEH_MIN_STATUS" + d + "CPLEX_VEH_MIN_EXE_TIME_MIN" + d
    + "CPLEX_MIN_SEC_OBJ" + d + "CPLEX_SEC_MIN_STATUS" + d + "CPLEX_SECOBJ_MIN_EXE_TIME_MIN" + d
    + "SOL_DISTANCE" + d + "SOL_TRAVEL_TIME" + d + "SOL_TOTAL_TIME" + d + "SOL_ENERGY" + d + "SOL_RECHARGING_TIME" + d + "SOL_RECHARGE_AMOUNT" + d + "NUM_OF_RECHARGES" + d+ "DIFF_SEC_CPLEX_AND_REAL"+d+ "SOL_USER_CONFIG" + "\n";
ofsFinal.Write(headerLine);

while (!ifs.EndOfStream)
{
    string line = ifs.ReadLine().Replace('.', ',');
    string[] splitLine = line.Split('\t');
    string solomonName = splitLine[0].ToUpper().Trim();
    string numCust = splitLine[1].ToUpper().Trim();
    bool skipProblem = true;
    //Branching whether the loaded problme is in specific or observed problem types (from cfg file)
    if (p.specificProblems.Count > 0)
    {
        foreach (Tuple<string, string> pair in p.specificProblems)
        {
            if (pair.Item1 == solomonName && pair.Item2 == numCust)
            {
                skipProblem = false;
                break;
            }
        }
    }
    else
    {
        if (p.observedNumCust.Contains(numCust))
        {
            foreach (string s in p.observedProblemType)
            {
                int ll = s.Trim().Length;
                if (s == "all" || solomonName.Substring(0, ll) == s.ToUpper().Trim())
                {
                    skipProblem = false;
                    break;
                }
            }
        }
    }
    if (skipProblem)
    {
        continue;
    }

    Console.WriteLine("Solving " + solomonName + "!");
    //Set BKS values from Schneider 2014
    p.BKSVehNum = Convert.ToInt32(splitLine[2].Trim());
    p.BKSCost = Convert.ToDouble(splitLine[3].Trim());
    p.loadInstanceSolomonEVRP(numCust, solomonName, pathCFG);
    //EVRPTWFR_solver4 sol2 = new EVRPTWFR_solver4(3, 7200, 7200, -1, p);
    Solver solver = new Solver(p);
    //OLD_EVRPTWFR_solver solver = new OLD_EVRPTWFR_solver(p);
    ofsFinal.Write(solver.outputLine.Replace(',', '.'));
    ofsFinal.Flush();

}

ifs.Close();
ofsFinal.Close();