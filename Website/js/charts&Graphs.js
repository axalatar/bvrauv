const motorGraph = document.getElementById("motorGraph");
const chartPID = document.getElementById("chartPID");
var desiredPID = "GET DESIRED PID VALUE FROM OTHER FILE";
var AUVactive = true; // Later get from a different file to tell if AUV is on

PID = new Chart(chartPID, {
  type: "line",
  data: {
    labels: ["Red", "Blue", "Yellow", "Green", "Purple", "Orange"],
    datasets: [{
      label: "PID Level",
      data: [12, 19, 3, 5, 2, 3],
      borderWidth: 1
    }]
  },
  options: {
    scales: {
      y: {
        min: -20,
        max: 20,
        title: {
          display: true,
          text: "Desired PID"
        },
        grid: {
          color: 'rgba(0, 0, 0, 0.1)'
        }
      }
    }
  }
});

/*while (AUVactive){
  PID.data = [12, 19, 3, 5, 2, 3]

  PID.update()
}*/