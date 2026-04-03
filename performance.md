| Pair               | Contacts C/N | Coll C (µs) | Coll N (µs) | Coll Win       | Res C (µs) | Res N (µs) | Res Win        |
|--------------------|-------------|------------|------------|----------------|-----------|-----------|----------------|
| circle-circle      | 1/1         | 3.67       | 5.54       | current 1.51x  | 4.63      | 10.49     | current 2.27x  |
| box-box            | 2/2         | 10.01      | 8.26       | new 1.21x      | 8.60      | 15.34     | current 1.78x  |
| capsule-capsule    | 1/2         | 5.97       | 6.13       | tie            | 4.94      | 16.61     | current 3.36x  |
| segment-segment    | 1/2         | 8.55       | 6.05       | new 1.41x      | 4.80      | 15.74     | current 3.28x  | skipped
| circle-box         | 1/1         | 5.67       | 5.69       | tie            | 5.06      | 10.87     | current 2.15x  |
| circle-capsule     | 1/1         | 4.66       | 5.76       | current 1.23x  | 4.81      | 10.56     | current 2.20x  |
| circle-segment     | 1/1         | 4.08       | 4.88       | current 1.20x  | 4.87      | 10.95     | current 2.25x  |
| box-segment        | 2/2         | 10.90      | 8.23       | new 1.32x      | 7.87      | 15.55     | current 1.98x  |
| box-capsule        | 2/2         | 9.82       | 8.65       | new 1.14x      | 8.11      | 16.55     | current 2.04x  |
| capsule-segment    | 1/2         | 5.50       | 8.03       | current 1.46x  | 4.95      | 15.65     | current 3.16x  |

Collision wins  | current=4 | new=4 | ties=2

Resolution wins | current=10 | new=0 | ties=0

Contact counts | mismatches=capsule-capsule (1/2), segment-segment (1/2), capsule-segment (1/2)