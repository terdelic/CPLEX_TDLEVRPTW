using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CPLEX_TDTSPTW
{
    public class Customer : User
    {
        public int _custID;
        public Customer(int userId, int custID, double x, double y, double demand, double etw, double ltw, double stw, Params p) :
            base(userId, x, y, demand, etw, ltw, stw, UserType.Customer, p)
        {
            this._custID = custID;
        }

        public Customer(Customer c) : base(c._userID, c.x, c.y, c.demand, c.etw, c.ltw, c.serviceTime, UserType.Customer, c.p)
        {
        }
    }
}
