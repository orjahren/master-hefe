# Manual scenario stats

This folder tracks the stats from the [prod scenario
logs](../prod/consolidated.log), for use in the thesis.

Method:

- Copy the prod log file
- Process each scenario entry sequentially. Track its type and remove it from
  the copied file

## Stats

### Unexpected keyword argument

- CE-2
- CE-3
- CE-4
- CE-5
- CE-6
- CE-7
- CE-8
- CE-9

Sum: 8

### Hard Carla crash

- junction
- junction-E
- junction-E2

Sum: 3

(Sikkert pga problemer med scenario runner)

### Illegal object spawn

- route-obstacles-E

Sum: 1

### Illegal imports

- Follow-e-6
- Follow-e-7
- route-obstacles-E-4

Sum: 3

### Funker fint

- follow
- follow-e
- follow-e-2
- Follow-e-3
- route-obstacles
- Follow-e-4
- Follow-e-5
- route-obstacles-E-2
- route-obstacles-E-3

Sum: 9

---

Totally 24 enhanced scenarios.

### Bases

- Route obstacles
- Follow vehicle
- Junction
- CutIn

Tot 4 bases

### Failure ratio

$(24-9)/24=63\%$
