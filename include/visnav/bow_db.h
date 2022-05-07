/**
BSD 3-Clause License

Copyright (c) 2018, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <fstream>

#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_vector.h>

#include <visnav/common_types.h>
#include <visnav/serialization.h>

bool comp(const pair<visnav::FrameCamId, double>& a,
          const pair<visnav::FrameCamId, double>& b) {
  return a.second < b.second;
}

namespace visnav {

class BowDatabase {
 public:
  BowDatabase() {}

  inline void insert(const FrameCamId& fcid, const BowVector& bow_vector) {
    // TODO SHEET 3: add a bow_vector that corresponds to frame fcid to the
    // inverted index. You can assume the image hasn't been added before.
    UNUSED(fcid);
    UNUSED(bow_vector);
    for (size_t i = 0; i < bow_vector.size(); i++) {
      int id = bow_vector[i].first;
      double weight = bow_vector[i].second;

      if (inverted_index.count(id) == 0) {
        tbb::concurrent_vector<std::pair<FrameCamId, WordValue>> fcid_weight;
        fcid_weight.push_back(make_pair(fcid, weight));
        inverted_index.emplace(id, fcid_weight);
      } else
        inverted_index[id].push_back(make_pair(fcid, weight));
    }
  }

  inline void query(const BowVector& bow_vector, size_t num_results,
                    BowQueryResult& results) const {
    // TODO SHEET 3: find num_results closest matches to the bow_vector in the
    // inverted index. Hint: for good query performance use std::unordered_map
    // to accumulate scores and std::partial_sort for getting the closest
    // results. You should use L1 difference as the distance measure. You can
    // assume that BoW descripors are L1 normalized.
    UNUSED(bow_vector);
    UNUSED(num_results);
    UNUSED(results);

    // First go through the word_id, then store the fcid in which the word_id
    // has shown up and use the corresponding weight in this fcid to compute the
    // score. fcid and score are stored in the map fcid_score_map.
    unordered_map<FrameCamId, double> fcid_score_map;
    for (size_t i = 0; i < bow_vector.size(); i++) {
      int wordid = bow_vector[i].first;

      for (size_t j = 0; j < inverted_index.at(wordid).size(); j++) {
        FrameCamId fcid = inverted_index.at(wordid)[j].first;

        double weight = inverted_index.at(wordid)[j].second;
        double query_weight = bow_vector[i].second;
        double score = abs(query_weight - weight) - query_weight - weight;
        if (fcid_score_map.count(fcid) == 0)
          fcid_score_map[fcid] = score + 2;
        else
          fcid_score_map[fcid] += score;
      }
    }

    // Do partial sort. If num_results is bigger than the size of the result
    // vector, then simply return the whole sorted vector.
    vector<pair<FrameCamId, double>> fcid_score(fcid_score_map.begin(),
                                                fcid_score_map.end());
    if (num_results > fcid_score.size()) num_results = fcid_score.size();
    partial_sort(fcid_score.begin(), fcid_score.begin() + num_results,
                 fcid_score.end(), comp);

    for (size_t i = 0; i < num_results; i++) {
      results.push_back(fcid_score[i]);
    }
  }

  void clear() { inverted_index.clear(); }

  void save(const std::string& out_path) {
    BowDBInverseIndex state;
    for (const auto& kv : inverted_index) {
      for (const auto& a : kv.second) {
        state[kv.first].emplace_back(a);
      }
    }
    std::ofstream os;
    os.open(out_path, std::ios::binary);
    cereal::JSONOutputArchive archive(os);
    archive(state);
  }

  void load(const std::string& in_path) {
    BowDBInverseIndex inverseIndexLoaded;
    {
      std::ifstream os(in_path, std::ios::binary);
      cereal::JSONInputArchive archive(os);
      archive(inverseIndexLoaded);
    }
    for (const auto& kv : inverseIndexLoaded) {
      for (const auto& a : kv.second) {
        inverted_index[kv.first].emplace_back(a);
      }
    }
  }

  const BowDBInverseIndexConcurrent& getInvertedIndex() {
    return inverted_index;
  }

 protected:
  BowDBInverseIndexConcurrent inverted_index;
};

}  // namespace visnav
