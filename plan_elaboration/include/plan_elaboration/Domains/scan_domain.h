#ifndef SCAN_DOMAIN_H
#define SCAN_DOMAIN_H

#include "plan_elaboration/virtual_domain.h"


class ScanDomain: public VirtualDomain
{
public:
    ScanDomain(ros::NodeHandle* node);
    ~ScanDomain() {};
    virtual std::vector<toaster_msgs::Fact> computeSpecificFacts(std::vector<toaster_msgs::Fact> facts);

private:
    std::string areaScan_; /**< Name of the area for scan*/
    std::vector<toaster_msgs::Fact> computeScanArea(std::vector<toaster_msgs::Fact> facts);
};

#endif // SCAN_DOMAIN_H
