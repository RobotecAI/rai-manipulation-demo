
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "RAIManipulationDemoSystemComponent.h"

#include <RAIManipulationDemo/RAIManipulationDemoTypeIds.h>

namespace RAIManipulationDemo
{
    class RAIManipulationDemoModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(RAIManipulationDemoModule, RAIManipulationDemoModuleTypeId, AZ::Module);
        AZ_CLASS_ALLOCATOR(RAIManipulationDemoModule, AZ::SystemAllocator);

        RAIManipulationDemoModule()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                RAIManipulationDemoSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<RAIManipulationDemoSystemComponent>(),
            };
        }
    };
}// namespace RAIManipulationDemo

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), RAIManipulationDemo::RAIManipulationDemoModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_RAIManipulationDemo, RAIManipulationDemo::RAIManipulationDemoModule)
#endif
