using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CPLEX_TDTSPTW
{
    /* Customer class used to store customer data. This class inherits User class, and all its atributs and public methods
     * It has one additional attribute custID, which is actually never used in this code
     */
    public class Customer : User
    {
        public int _custID;
        //Constructor with all passed parameters for customer
        public Customer(int userId, int custID, double x, double y, double demand, double etw, double ltw, double stw, Params p) :
            base(userId, x, y, demand, etw, ltw, stw, UserType.Customer, p)
        {
            this._custID = custID;
        }
        //Constructor to create an instance of another customer - this is used for creatign depot instances
        public Customer(Customer c) : base(c._userID, c.x, c.y, c.demand, c.etw, c.ltw, c.serviceTime, UserType.Customer, c.p)
        {
        }
    }
}
