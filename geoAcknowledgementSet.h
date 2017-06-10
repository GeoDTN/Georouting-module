/*
/*
 * geoAcknowledgementSet.h
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

#ifndef GEOACKNOWLEDGEMENTSET_H_
#define GEOACKNOWLEDGEMENTSET_H_

#include "routing/NodeHandshake.h"
#include <ibrdtn/data/BundleList.h>
#include <ibrcommon/thread/Mutex.h>
#include <iostream>
#include <set>

namespace dtn
{
	namespace routing
	{
		/*!
		 * \brief Set of geoAcknowledgements, that can be serialized in node handshakes.
		 */
		class geoAcknowledgementSet : public NodeHandshakeItem, public ibrcommon::Mutex
		{
		public:
			geoAcknowledgementSet();
			geoAcknowledgementSet(const geoAcknowledgementSet&);

			/**
			 * Add a bundle to the set
			 */
			void add(const dtn::data::MetaBundle &bundle) throw ();

			/**
			 * Expire outdated entries
			 */
			void expire(const dtn::data::Timestamp &timestamp) throw ();

			/**
			 * merge the set with a second geoAcknowledgementSet
			 */
			void merge(const geoAcknowledgementSet&) throw ();

			/**
			 * Returns true if the given bundleId is in the bundle list
			 */
			bool has(const dtn::data::BundleID &id) const throw ();

			/* virtual methods from NodeHandshakeItem */
			virtual const dtn::data::Number& getIdentifier() const; ///< \see NodeHandshakeItem::getIdentifier
			virtual dtn::data::Length getLength() const; ///< \see NodeHandshakeItem::getLength
			virtual std::ostream& serialize(std::ostream& stream) const; ///< \see NodeHandshakeItem::serialize
			virtual std::istream& deserialize(std::istream& stream); ///< \see NodeHandshakeItem::deserialize
			static const dtn::data::Number identifier;

			friend std::ostream& operator<<(std::ostream&, const geoAcknowledgementSet&);
			friend std::istream& operator>>(std::istream&, geoAcknowledgementSet&);

			typedef dtn::data::BundleList::const_iterator const_iterator;
			const_iterator begin() const { return _bundles.begin(); };
			const_iterator end() const { return _bundles.end(); };

		private:
			dtn::data::BundleList _bundles;
		};
	} /* namespace routing */
} /* namespace dtn */
#endif /* ACKNOWLEDGEMENTSET_H_ */
