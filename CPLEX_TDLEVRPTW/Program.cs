using CPLEX_TDTSPTW;
using System.Globalization;

//Culture is included to get the directory where config and data is located
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
Params p = new Params(pathCFG + "cfg.txt");


//Stream for output file
StreamWriter ofsFinal = new StreamWriter(p.outputFileName + "_" + p.minimizationType.ToString() + "_" + p.travelTimeCompType + ".txt");
string st = "new";
//StreamWriter help=new StreamWriter($"VarsAnalysis_{st}.txt");

//help.WriteLine($"NUM_VARS_{st}\tNUM_CONSTRAINTS_{st}");

//Stream for input file containing all small instances and values from Schneider (2014)
StreamReader ifs = new StreamReader(pathCFG + p.instancesFile);
//Delimiter
string d = ";";
//Header in output line
string headerLine = "INSATNCE_NAME" + d + "NUM_CUST" + d + "SCHNEIDER_VEH" + d + "SCHNEIDER_DIST" + d + "BETA" + d +
    "KNOWN_VEH_NUM_CPLEX" + d + "MINIMIZATION_TYPE" + d + "CPLEX_MIN_VEH_NUM" + d + "CPLEX_VEH_MIN_STATUS" + d + "CPLEX_VEH_MIN_EXE_TIME_MIN" + d
    + "CPLEX_MIN_SEC_OBJ" + d + "CPLEX_SEC_MIN_STATUS" + d + "CPLEX_SECOBJ_MIN_EXE_TIME_MIN" + d
    + "SOL_DISTANCE" + d + "SOL_TRAVEL_TIME" + d + "SOL_TOTAL_TIME" + d + "SOL_ENERGY" + d + "SOL_RECHARGING_TIME" + d + "SOL_RECHARGE_AMOUNT" + d + "NUM_OF_RECHARGES" + d+ "DIFF_SEC_CPLEX_AND_REAL"+d+ "SOL_USER_CONFIG" + "\n";
ofsFinal.Write(headerLine);

while (!ifs.EndOfStream)
{
    //Parse one line from small instance file
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
    //Set BKS values from Schneider (2014)
    p.BKSVehNum = Convert.ToInt32(splitLine[2].Trim());
    p.BKSCost = Convert.ToDouble(splitLine[3].Trim());
    //Load problem instance
    p.loadInstanceSolomonEVRP(numCust, solomonName, pathCFG);
    //Run solver
    Solver solver = new Solver(p);
    ofsFinal.Write(solver.outputLine.Replace(',', '.')); //On my computer for float precision, by default a ',' is used, and I want to use '.'
    ofsFinal.Flush(); //Flush to have the latest result written in the file, just for the case if the optimization breaks (i.e. power outage)
    //help.WriteLine($"{solver.numVars}\t{solver.numConstraints}");
}
//Close streams
ifs.Close();
ofsFinal.Close();
//help.Close();

//Sending an email to myslef that the optimization is over
string machineName = System.Environment.MachineName;
//Misc.SendEmail("Finished\n, Machine name=" + machineName + ", File Name=\n" + p.outputFileName + "\n, MinimizationType=" + p.minimizationType.ToString() + "\n, Travel Time Coeffs=" + p.travelTimeCompType);
//Console.WriteLine("Finished!!!");
Console.ReadKey();