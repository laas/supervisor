#ifndef SCAN_US_DOMAIN_H
#define SCAN_US_DOMAIN_H

#include "plan_elaboration/virtual_domain.h"


class ScanUsDomain: public VirtualDomain
{
public:
    ScanUsDomain(ros::NodeHandle* node);
    ~ScanUsDomain() {};
    virtual std::vector<toaster_msgs::Fact> computeSpecificFacts(std::vector<toaster_msgs::Fact> facts);

private:
    std::map<std::string, bool> objectsScanned_;
    void computeScanned(std::vector<toaster_msgs::Fact> facts);
    std::vector<toaster_msgs::Fact> computeHasInHand(std::vector<toaster_msgs::Fact> facts);
    std::vector<toaster_msgs::Fact> computeHasOn(std::vector<toaster_msgs::Fact> facts);
};

#endif // SCAN_US_DOMAIN_H
