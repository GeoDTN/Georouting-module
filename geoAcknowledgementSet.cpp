/*
/*
 * geoAcknowledgementSet.cpp
 *
 * Copyright (C) 2015 TNO
 *
 * Written-by: Tadewos Somano <Tadewos.SomanoEwalo@tno.nl,tadewos85@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "routing/geoprophet/geoAcknowledgementSet.h"
#include "core/BundleCore.h"
#include <ibrdtn/utils/Clock.h>

namespace dtn
{
	namespace routing
	{
		const dtn::data::Number geoAcknowledgementSet::identifier = NodeHandshakeItem::GEOPROPHET_ACKNOWLEDGEMENT_SET;

		geoAcknowledgementSet::geoAcknowledgementSet()
		{
		}

		geoAcknowledgementSet::geoAcknowledgementSet(const geoAcknowledgementSet &other)
		 : ibrcommon::Mutex(), _bundles(other._bundles)
		{
		}

		void geoAcknowledgementSet::add(const dtn::data::MetaBundle &bundle) throw ()
		{
			try {
				// check if the bundle is valid
				dtn::core::BundleCore::getInstance().validate(bundle);

				// add the bundle to the set
				_bundles.add(bundle);
			} catch (dtn::data::Validator::RejectedException &ex) {
				// bundle rejected
			}
		}

		void geoAcknowledgementSet::expire(const dtn::data::Timestamp &timestamp) throw ()
		{
			_bundles.expire(timestamp);
		}

		void geoAcknowledgementSet::merge(const geoAcknowledgementSet &other) throw ()
		{
			for(dtn::data::BundleList::const_iterator it = other._bundles.begin(); it != other._bundles.end(); ++it)
			{
				const dtn::data::MetaBundle &ack = (*it);
				this->add(ack);
			}
		}

		bool geoAcknowledgementSet::has(const dtn::data::BundleID &id) const throw ()
		{
			dtn::data::BundleList::const_iterator iter = _bundles.find(dtn::data::MetaBundle::create(id));
			return !(iter == _bundles.end());
		}

		const dtn::data::Number& geoAcknowledgementSet::getIdentifier() const
		{
			return identifier;
		}

		dtn::data::Length geoAcknowledgementSet::getLength() const
		{
			std::stringstream ss;
			serialize(ss);
			return ss.str().length();
		}

		std::ostream& geoAcknowledgementSet::serialize(std::ostream& stream) const
		{
			stream << (*this);
			return stream;
		}

		std::istream& geoAcknowledgementSet::deserialize(std::istream& stream)
		{
			stream >> (*this);
			return stream;
		}

		std::ostream& operator<<(std::ostream& stream, const geoAcknowledgementSet& ack_set)
		{
			dtn::data::Number ackset_size(ack_set._bundles.size());
			ackset_size.encode(stream);
			for (dtn::data::BundleList::const_iterator it = ack_set._bundles.begin(); it != ack_set._bundles.end(); ++it)
			{
				const dtn::data::MetaBundle &ack = (*it);
				stream << (const dtn::data::BundleID&)ack;
				ack.expiretime.encode(stream);
				ack.lifetime.encode(stream);
			}

			return stream;
		}

		std::istream& operator>>(std::istream &stream, geoAcknowledgementSet &ack_set)
		{
			// clear the geoack set first
			ack_set._bundles.clear();

			dtn::data::Number size;
			size.decode(stream);

			for(size_t i = 0; size > i; ++i)
			{
				dtn::data::MetaBundle ack;
				stream >> (dtn::data::BundleID&)ack;
				ack.expiretime.decode(stream);
				ack.lifetime.decode(stream);

				ack_set.add(ack);
			}
			return stream;
		}
	} /* namespace routing */
} /* namespace dtn */
