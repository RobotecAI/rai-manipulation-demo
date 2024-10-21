
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "RAIManipulationDemoSystemComponent.h"

namespace RAIManipulationDemo
{
    void RAIManipulationDemoSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<RAIManipulationDemoSystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<RAIManipulationDemoSystemComponent>("RAIManipulationDemo", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void RAIManipulationDemoSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("RAIManipulationDemoService"));
    }

    void RAIManipulationDemoSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("RAIManipulationDemoService"));
    }

    void RAIManipulationDemoSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void RAIManipulationDemoSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    RAIManipulationDemoSystemComponent::RAIManipulationDemoSystemComponent()
    {
        if (RAIManipulationDemoInterface::Get() == nullptr)
        {
            RAIManipulationDemoInterface::Register(this);
        }
    }

    RAIManipulationDemoSystemComponent::~RAIManipulationDemoSystemComponent()
    {
        if (RAIManipulationDemoInterface::Get() == this)
        {
            RAIManipulationDemoInterface::Unregister(this);
        }
    }

    void RAIManipulationDemoSystemComponent::Init()
    {
    }

    void RAIManipulationDemoSystemComponent::Activate()
    {
        RAIManipulationDemoRequestBus::Handler::BusConnect();
    }

    void RAIManipulationDemoSystemComponent::Deactivate()
    {
        RAIManipulationDemoRequestBus::Handler::BusDisconnect();
    }
}
