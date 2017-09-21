/* Autogenerated with kurento-module-creator */

#include <gst/gst.h>
#include "MediaPipeline.hpp"
#include <HeartOfAsiaImplFactory.hpp>
#include "HeartOfAsiaImpl.hpp"
#include <jsonrpc/JsonSerializer.hpp>
#include <KurentoException.hpp>
#include "MediaPipelineImpl.hpp"

#define GST_CAT_DEFAULT kurento_heart_of_asia_impl
GST_DEBUG_CATEGORY_STATIC (GST_CAT_DEFAULT);
#define GST_DEFAULT_NAME "KurentoHeartOfAsiaImpl"

namespace kurento
{
namespace module
{
namespace heartofasia
{

HeartOfAsiaImpl::HeartOfAsiaImpl (const boost::property_tree::ptree &config, std::shared_ptr<MediaPipeline> mediaPipeline) : OpenCVFilterImpl (config, std::dynamic_pointer_cast<MediaPipelineImpl> (mediaPipeline) )

{
}

MediaObjectImpl *
HeartOfAsiaImplFactory::createObject (const boost::property_tree::ptree &config, std::shared_ptr<MediaPipeline> mediaPipeline) const
{
  return new HeartOfAsiaImpl (config, mediaPipeline);
}

HeartOfAsiaImpl::StaticConstructor HeartOfAsiaImpl::staticConstructor;

HeartOfAsiaImpl::StaticConstructor::StaticConstructor()
{
  GST_DEBUG_CATEGORY_INIT (GST_CAT_DEFAULT, GST_DEFAULT_NAME, 0,
                           GST_DEFAULT_NAME);
}

} /* heartofasia */
} /* module */
} /* kurento */
